// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.telescope;

import static org.wpilib.units.Units.Seconds;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.util.Units;

import first.robot.subsystems.telescope.TelescopeConstants.ArmConstants;
import first.robot.subsystems.telescope.TelescopeConstants.PivotConstants;
import first.robot.subsystems.telescope.TelescopeConstants.TelescopeStates;
import first.robot.util.LoggedTunableNumber;

public class Telescope implements Mechanism {
    private final TelescopeIO io;

    private final TelescopeIOInputsAutoLogged inputs = new TelescopeIOInputsAutoLogged();

    @AutoLogOutput(key="Mechanisms/Telescope/State") private TelescopeStates state = TelescopeStates.HOME;

    @AutoLogOutput(key="Mechanisms/Telescope/Arm/In Climb Sequence") private boolean isClimbing = false;
    
    @AutoLogOutput(key="Mechanisms/Telescope/Pivot/Raw Setpoint") private double rawAngle = 0.0;
    @AutoLogOutput(key="Mechanisms/Telescope/Pivot/Adjust") private double angleAdjust = 0.0;
    @AutoLogOutput(key="Mechanisms/Telescope/Pivot/True Setpoint") private double trueAngle = 0.0;
    private final LoggedTunableNumber tunableAngle = new LoggedTunableNumber("Telescope/Pivot/Angle Setpoint Deg", 0.0);
    
    @AutoLogOutput(key="Mechanisms/Telescope/Arm/Raw Setpoint") private double rawExtension = 0.0;
    @AutoLogOutput(key="Mechanisms/Telescope/Arm/Adjust") private double extensionAdjust = 0.0;
    @AutoLogOutput(key="Mechanisms/Telescope/Arm/True Setpoint") private double trueExtension = 0.0;
    private final LoggedTunableNumber tunableExtension = new LoggedTunableNumber("Telescope/Arm/Extension Setpoint In", 0.0);

    /** Creates a new Telescope. */
    public Telescope(TelescopeIO io) {
        this.io = io;
    }

    public void logIO() {
        io.updateInputs(inputs);
        Logger.processInputs("Telescope", inputs);
        Logger.recordOutput("Mechanisms/Telescope/Pivot/Setpoint Angle Deg", state.pivotAngleDeg);
        Logger.recordOutput("Mechanisms/Telescope/Pivot/Angle Deg", getPivotAngleDeg());
        Logger.recordOutput("Mechanisms/Telescope/Pivot/Abs Encoder Angle Deg", Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition));

        Logger.recordOutput("Mechanisms/Telescope/Arm/Extension Inches", getArmExtensionInches());
        Logger.recordOutput("Mechanisms/Telescope/Arm/Setpoint Extension Inches", getArmSetpoint(state));
        Logger.recordOutput("Mechanisms/Telescope/Arm/Dog Shifter PW", inputs.armServoAppliedPulseWidth);
    }

    public Command applyState(TelescopeStates state) {
        return run(co -> {
            this.state = state;
            if (isClimbing && state != TelescopeStates.CLUMB) {
                co.await(shiftDogs(false));
            } // if the driver switches away from climbing it un-engages the dog shifter to make sure extension still works properly
            // normal logic, will complete naturally
            if (state != TelescopeStates.TUNING) {
                Debouncer setpointDebouncer = new Debouncer(0.2, DebounceType.FALLING);
                    rawAngle = state.pivotAngleDeg;
                    rawExtension = state.armExtensionInches;
                        trueAngle = Math.clamp(rawAngle + angleAdjust, PivotConstants.MIN_ANGLE_DEG, PivotConstants.MAX_ANGLE_DEG);
                        trueExtension = Math.clamp(rawExtension + extensionAdjust, Units.metersToInches(ArmConstants.MIN_EXTENSION_METERS), Units.metersToInches(ArmConstants.MAX_EXTENSION_METERS));
                while(setpointDebouncer.calculate(
                    Math.abs((state.pivotAngleDeg - Units.rotationsToDegrees(inputs.pivotAbsEncoderPosition))) > 0.5
                    || Math.abs(getArmSetpoint(state) - getArmExtensionInches()) > 0.05)
                ) {
                    // functions as a timer, cmd gives up control when it's close to its setpoint (within 0.5° and 0.05");
                    io.setPivotAngleDeg(trueAngle);
                    // io.setPivotAngleDeg(trueAngle, () -> 0.0);
                    io.setArmExtensionIn(state==TelescopeStates.CLUMB, trueExtension);
                    // io.setArmExtensionIn(state==TelescopeStates.CLUMB, trueExtension, () -> 0.0);
                    co.yield();
                }
                if (state == TelescopeStates.CLIMB_RAISED) {
                    co.await(shiftDogs(true));
                } // engages the dog shifter if the climb sequence is initiated
            }
            // tuning logic, will never complete naturally
            else {
                while(true) {
                    if(tunableAngle.hasChanged(tunableAngle.hashCode())) rawAngle = tunableAngle.get();
                    if(tunableExtension.hasChanged(tunableExtension.hashCode())) rawExtension = tunableExtension.get();
                    trueAngle = Math.clamp(rawAngle + angleAdjust, PivotConstants.MIN_ANGLE_DEG, PivotConstants.MAX_ANGLE_DEG);
                        trueExtension = Math.clamp(rawExtension + extensionAdjust, Units.metersToInches(ArmConstants.MIN_EXTENSION_METERS), Units.metersToInches(ArmConstants.MAX_EXTENSION_METERS));
                    io.setPivotAngleDeg(trueAngle);
                    io.setArmExtensionIn(isClimbing, trueExtension);
                    co.yield();
                }
            }
        }).named("TELE " + state.name());
    }

    public Command shiftDogs(boolean isClimbing) {
        return run(co -> {
            this.isClimbing = isClimbing;
            io.shiftDogs(isClimbing);
            co.wait(Seconds.of(ArmConstants.SHIFT_DURATION_SEC));
        }).named("SWITCH TO " + (isClimbing ? "CLIMB" : "EXTENSION"));
    }

    public Command adjustAngleDeg(double by) {
        return Command.noRequirements(co -> {
            while(true) {
                angleAdjust += by;
                io.setPivotAngleDeg(Math.clamp(rawAngle + angleAdjust, PivotConstants.MIN_ANGLE_DEG, PivotConstants.MAX_ANGLE_DEG));
                co.yield();
            }
        }).named("ADJUST TELE ANGLE");
    }
    
    public Command adjustExtensionIn(double by) {
        return Command.noRequirements(co -> {
            while(true) {
                extensionAdjust += by;
                io.setArmExtensionIn(state==TelescopeStates.CLUMB, Math.clamp(rawExtension + extensionAdjust, 0, Units.metersToInches(ArmConstants.MAX_EXTENSION_METERS)));
                co.yield();
            }
        }).named("ADJUST TELE EXTENSION");
    }

    public TelescopeStates getState() {
        return state;
    }

    public double getPivotAngleDeg() {
        return Units.rotationsToDegrees(mean(
            inputs.pivot1Data.position(),
            -inputs.pivot2Data.position(), // because this one follows in the opposed direction
            inputs.pivot3Data.position())
            / PivotConstants.REDUCTION
        );
    }

    public double getArmExtensionInches() {
        return mean(
            inputs.arm1Data.position(),
            inputs.arm2Data.position())
            / (isClimbing ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION)
            // / (inputs.armServoAppliedPulseWidth == ArmConstants.CLIMB_PULSE_WIDTH_uS ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION)
            // / ArmConstants.EXTENSION_REDUCTION
            * Units.metersToInches(ArmConstants.ROTOR_CIRCUMF_METERS)
            // + (state == TelescopeStates.CLUMB ? (
            //     Units.inchesToMeters(TelescopeStates.CLIMB_RAISED.getArmExtensionInches())
            //         / ArmConstants.ROTOR_CIRCUMF_METERS
            //         * ArmConstants.EXTENSION_REDUCTION)
            //     : 0)
        ;
    }

    private double getArmSetpoint(TelescopeStates state) {
        // return (state==TelescopeStates.CLUMB ? 2 : state.getArmExtensionInches());
        return state.armExtensionInches;
    }

    private double mean(double... values) {
        double sum = 0.0;
        for (double value : values) {
            sum += value;
        }
        return (sum / values.length);
    }
}
