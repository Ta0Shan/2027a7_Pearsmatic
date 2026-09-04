package first.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.Units;

import first.robot.subsystems.endEffector.EEConstants.WristStates;
import first.robot.util.LoggedTunableNumber;
import first.robot.Constants.CrystalColor;
import first.robot.subsystems.endEffector.EEConstants.RollerStates;

public class EE implements Mechanism {

    private final EEIO io;

    private final EEIOInputsAutoLogged inputs = new EEIOInputsAutoLogged();

    private WristStates wristState = WristStates.STOWED;

    @AutoLogOutput(key="Mechanisms/End Effector/Wrist/Raw Setpoint") private double rawAngle = 0.0;
    @AutoLogOutput(key="Mechanisms/End Effector/Wrist/Adjust") private double angleAdjust = 0.0;
    @AutoLogOutput(key="Mechanisms/End Effector/Wrist/True Setpoint") private double trueAngle = 0.0;
    private final LoggedTunableNumber tunableAngle = new LoggedTunableNumber("End Effector/Wrist/Angle Setpoint Deg", 0.0);
    
    @AutoLogOutput(key="Mechanisms/End Effector/Rollers/Raw Setpoint") private double rawVoltage = 0.0;
    @AutoLogOutput(key="Mechanisms/End Effector/Rollers/Adjust") private double voltageAdjust = 0.0;
    @AutoLogOutput(key="Mechanisms/End Effector/Rollers/True Setpoint") private double trueVoltage = 0.0;
    private final LoggedTunableNumber tunableVoltage = new LoggedTunableNumber("End Effector/Rollers/Voltage Setpoint", 0.0);

    private RollerStates rollerState = RollerStates.IDLE;

    public EE(EEIO io) {
        this.io = io;
    }

    public void logIO() {
        io.updateInputs(inputs);
        Logger.processInputs("End Effector", inputs);

        Logger.recordOutput("Mechanisms/End Effector/State", wristState.name() + " " + rollerState.name());
        Logger.recordOutput("Mechanisms/End Effector/Crystal Color", crystalColor().name());

        Logger.recordOutput("Mechanisms/End Effector/Wrist/Angle Deg", getWristAngleDeg());
        Logger.recordOutput("Mechanisms/End Effector/Wrist/Setpoint Deg", wristState.angleDeg);

        Logger.recordOutput("Mechanisms/End Effector/Rollers/Voltage Setpoint", rollerState.voltage);
        Logger.recordOutput("Mechanisms/End Effector/Rollers/Voltage", inputs.rollerData.appliedVolts());
        Logger.recordOutput("Mechanisms/End Effector/Rollers/RPS", getRollersRPS());
        Logger.recordOutput("Mechanisms/End Effector/Rollers/Surface Speed MPS", getRollersRPS() * EEConstants.ROLLER_CIRCUMF_METERS);
    }

    public Command applyState(WristStates wristState, RollerStates rollerState) {
        return run(co -> {
            this.wristState = wristState;
            this.rollerState = rollerState;
            // normal logic, will complete naturally
            if (!(wristState == WristStates.TUNING && rollerState == RollerStates.TUNING)) {
                Debouncer setpointDebouncer = new Debouncer(0.2);
                    rawAngle = wristState.angleDeg;
                    rawVoltage = rollerState.voltage;
                        trueAngle = Math.clamp(rawAngle + angleAdjust, EEConstants.MIN_ANGLE_DEG, EEConstants.MAX_ANGLE_DEG);
                        trueVoltage = (rollerState==RollerStates.IDLE ? 0.0 : Math.clamp(rawVoltage + voltageAdjust, -12, 12));
                while(setpointDebouncer.calculate(
                    Math.abs(wristState.angleDeg - Units.rotationsToDegrees(inputs.wristData.position()) / EEConstants.WRIST_REDUCTION) > 0.5)
                ) {
                    // functions as a timer, cmd gives up control when it's close to its setpoint (within 0.5°)
                    io.setWristAngleDeg(trueAngle);
                    // io.setWristAngleDeg(trueAngle, () -> 0.0);
                    io.setRollerVoltage(trueVoltage);
                    co.yield();
                }
            }
            // tuning logic, will not complete naturally
            else {
                while(true) {
                    if (tunableAngle.hasChanged(tunableAngle.hashCode())) rawAngle = tunableAngle.get();
                    if (tunableVoltage.hasChanged(tunableVoltage.hashCode())) rawVoltage = tunableVoltage.get();
                    trueAngle = Math.clamp(rawAngle + angleAdjust, EEConstants.MIN_ANGLE_DEG, EEConstants.MAX_ANGLE_DEG);
                    trueVoltage = (Math.clamp(rawVoltage + voltageAdjust, -12, 12));
                    io.setWristAngleDeg(trueAngle);
                    io.setRollerVoltage(trueVoltage);
                    co.yield();
                }
            }
        }).named("EE " + wristState.name() + " " + rollerState.name());
    }

    public Command applyState(WristStates wristState) {
        return applyState(wristState, rollerState);
    }
    public Command applyState(RollerStates rollerState) {
        return applyState(wristState, rollerState);
    }

    public Command adjustAngleDeg(double by) {
        return Command.noRequirements(co -> {
            while(true) {
                angleAdjust += by;
                io.setWristAngleDeg(Math.clamp(rawAngle + angleAdjust, EEConstants.MIN_ANGLE_DEG, EEConstants.MAX_ANGLE_DEG));
                co.yield();
            }
        }).named("ADJUST WRIST ANGLE");
    }

    public Command adjustVoltage(double by) {
        return Command.noRequirements(co -> {
            while(true) {
                voltageAdjust += by;
                if (rollerState!=RollerStates.IDLE) io.setRollerVoltage(Math.clamp(rawVoltage + voltageAdjust, -12, 12));
                co.yield();
            }
        }).named("ADJUST ROLLER VOLTAGE");
    }

    public WristStates getWristState() {
        return wristState;
    }

    public RollerStates getRollerState() {
        return rollerState;
    }

    public double getWristAngleDeg() {
        return Units.rotationsToDegrees(inputs.wristData.position()) / EEConstants.WRIST_REDUCTION;
    }

    public double getRollersRPS() {
        return inputs.rollerData.velocity() / EEConstants.ROLLER_REDUCTION;
    }

    @AutoLogOutput(key="Mechanisms/End Effector/Has Crystal")
    public boolean hasCrystal() {
        return inputs.colorReading != CrystalColor.NONE;
    }

    public CrystalColor crystalColor() {
        return inputs.colorReading;
    }

    public Command setCrystalColor(CrystalColor color) {
        return Command.noRequirements(co -> {
            inputs.colorReading = color;
        }).named("SET COLOR " + color.name());
    }
}
