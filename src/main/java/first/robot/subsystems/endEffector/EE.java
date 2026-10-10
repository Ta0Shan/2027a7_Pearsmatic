package first.robot.subsystems.endEffector;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.util.Units;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

import first.robot.util.LoggedTunableNumber;
import first.robot.Constants.CrystalColor;
import first.robot.subsystems.endEffector.EEConstants.EEInputs;
import first.robot.subsystems.endEffector.EEConstants.RollerStates;
import first.robot.subsystems.endEffector.EEConstants.WristStates;
import first.robot.subsystems.endEffector.EEIO.EEIOInputs;

public class EE implements Mechanism {

    private final EEIO io;

    private EEInputs inputs = new EEInputs();

    private WristStates wristState = WristStates.STOWED;

    private double rawAngle = 0.0;
    private double angleAdjust = 0.0;
    private double trueAngle = 0.0;
    
    private double rawVoltage = 0.0;
    private double voltageAdjust = 0.0;
    private double trueVoltage = 0.0;

    private RollerStates rollerState = RollerStates.IDLE;

    private final TunableDouble angleTuner = TunableDouble.create(0.0);
    private final TunableDouble voltageTuner = TunableDouble.create(0.0);

    public EE(EEIO io) {
        this.io = io;

        Tunables.publish("Wrist tunable setpoint", angleTuner);
        Tunables.publish("Rollers tunable setpoint", voltageTuner);
    }

    public void logIO() {
        inputs = io.updateInputs();
        Telemetry.log("Inputs/EE", inputs);

        Telemetry.log("Mechanisms/States/End Effector", wristState.name() + " " + rollerState.name());

        Telemetry.log("Mechanisms/Wrist/Crystal", crystalColor().name());
        Telemetry.log("Mechanisms/Wrist/Raw Setpoint", rawAngle);
        Telemetry.log("Mechanisms/Wrist/Adjust", angleAdjust);
        Telemetry.log("Mechanisms/Wrist/True Setpoint Deg", trueAngle);
        Telemetry.log("Mechanisms/Wrist/Angle Deg", getWristAngleDeg());

        Telemetry.log("Mechanisms/Rollers/Raw Setpoint", rawVoltage);
        Telemetry.log("Mechanisms/Rollers/Adjust", voltageAdjust);
        Telemetry.log("Mechanisms/Rollers/True Setpoint V", trueVoltage);
        Telemetry.log("Mechanisms/Rollers/Voltage", inputs.rollerData().appliedVolts());
        Telemetry.log("Mechanisms/Rollers/RPS", getRollersRPS());
        Telemetry.log("Mechanisms/Rollers/Surface Speed MPS", getRollersRPS() * EEConstants.ROLLER_CIRCUMF_METERS);
    }

    public Command applyState(WristStates wristState, RollerStates rollerState) {
        return run(co -> {
            this.wristState = wristState;
            this.rollerState = rollerState;
            // normal logic, will complete naturally
            if (!(wristState == WristStates.TUNING) && !(rollerState == RollerStates.TUNING)) {
                Debouncer setpointDebouncer = new Debouncer(0.2, DebounceType.FALLING);
                    rawAngle = wristState.angleDeg;
                    rawVoltage = rollerState.voltage;
                        trueAngle = Math.clamp(rawAngle + angleAdjust, EEConstants.MIN_ANGLE_DEG, EEConstants.MAX_ANGLE_DEG);
                        trueVoltage = (rollerState==RollerStates.IDLE ? 0.0 : Math.clamp(rawVoltage + voltageAdjust, -12, 12));
                while(setpointDebouncer.calculate(
                    Math.abs(trueAngle - Units.rotationsToDegrees(inputs.wristData().position()) / EEConstants.WRIST_REDUCTION) > 0.5)
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
                    rawAngle = angleTuner.get();
                    rawVoltage = voltageTuner.get();
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
        return Units.rotationsToDegrees(inputs.wristData().position()) / EEConstants.WRIST_REDUCTION;
    }

    public double getRollersRPS() {
        return inputs.rollerData().velocity() / EEConstants.ROLLER_REDUCTION;
    }

    public boolean hasCrystal() {
        return inputs.colorReading() != CrystalColor.NONE;
    }

    public CrystalColor crystalColor() {
        return inputs.colorReading();
    }

    public Command setCrystalColor(CrystalColor color) {
        return Command.noRequirements(co -> {
            EEConstants.simCrystalColor = color;
        }).named("SET COLOR " + color.name());
    }
}
