// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.subsystems.telescope;

import org.wpilib.math.util.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import first.robot.Constants;
import first.robot.subsystems.launcher.LauncherConstants;

/** Add your docs here. */
public class TelescopeConstants {

    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;

    public static enum TelescopeStates {
        HOME(PivotConstants.STARTING_ANGLE_DEG, 0.), // 0 deg
        DOWN(0., 0.), // 0 deg
        L1_FRONT(35., 0.), // 35 deg
        L2_FRONT(35., 25.), // 50 deg
        L1_BACK(138., 0.), // 127.5 deg
        L2_BACK(138., 23.), // 127.5 deg
        CLASSIFIER_FRONT(70., 0.),
        CLASSIFIER_BACK(90., 0.),
        CLIMB_RAISED(90., 7.), // 90 deg
        CLUMB(90., 2.), // 90 deg
        LAUNCHER(0., 0.), // 0 deg
        TUNING(0, 0); // tuning
        ;

        public final double pivotAngleDeg;
        public final double armExtensionInches;

        private TelescopeStates(double pivotAngleDeg, double armExtensionInches) {
            this.pivotAngleDeg = pivotAngleDeg;
            this.armExtensionInches = armExtensionInches;
        }
    }

    public class PivotConstants {

        public static final int PIVOT_1_ID = 21;
        public static final int PIVOT_2_ID = 22;
        public static final int PIVOT_3_ID = 23;

        public static final double REDUCTION = (48./12.) * (48./16.) * (50./12.) * (52./20.);
        
        public static final TalonFXConfiguration PIVOT_CONFIG() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotionMagic.MotionMagicCruiseVelocity = 100;
            config.MotionMagic.MotionMagicAcceleration = 150;
            config.MotionMagic.MotionMagicJerk = 300;

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 60;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 60;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            config.Slot0.kP = 1.0;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0; // TODO: tune

            return config;
        }
        public static final double STARTING_ANGLE_DEG = 60.0;
        
        public static final int CANCODER_ID = 24;
        public static final double CANCODER_OFFSET = 0.0;

        public static final CANcoderConfiguration CANCODER_CONFIG() {
            CANcoderConfiguration config = new CANcoderConfiguration();

            config.MagnetSensor.MagnetOffset = CANCODER_OFFSET;
            config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

            return config;
        }

        public static final double MIN_ANGLE_DEG = 0.0;
        public static final double MAX_ANGLE_DEG = 160.0;

    }

    public class ArmConstants {
        public static final int ARM_1_ID = 31;
        public static final int ARM_2_ID = 32;
        
        public static final double BASE_REDUCTION = (40./50.) * (50/80.) * (48./12.) * (30./30.) * (30./28.);
        public static final double EXTENSION_REDUCTION = BASE_REDUCTION * (30./40.) * (40./16.);
        public static final double CLIMB_REDUCTION = BASE_REDUCTION * (50./20.) * (40./16.);

        public static final double ROTOR_CIRCUMF_METERS = Units.inchesToMeters(Math.PI * (1.2871145)); // TODO: double check this number
        
        public static final int SERVO_HUB_ID = 33;
        public static final int EXTENSION_PULSE_WIDTH_uS = 2500; // TODO: should be max clockwise
        public static final int CLIMB_PULSE_WIDTH_uS = 500; // TODO: should be max counter-clockwise
        // the dog shifter only makes like a 90* turn so it should be relatively quick either way
        public static final double SHIFT_DURATION_SEC = 1.0;

        public static final TalonFXConfiguration CONFIG() {
            TalonFXConfiguration config = new TalonFXConfiguration();

            config.MotionMagic.MotionMagicCruiseVelocity = 100;
            config.MotionMagic.MotionMagicAcceleration = 150;
            // config.MotionMagic.MotionMagicJerk = 300; // TODO: tune

            config.CurrentLimits.StatorCurrentLimitEnable = true;
            config.CurrentLimits.StatorCurrentLimit = 60;
            config.CurrentLimits.SupplyCurrentLimitEnable = true;
            config.CurrentLimits.SupplyCurrentLimit = 60;

            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

            config.Slot0.kP = 1.0;
            config.Slot0.kI = 0.0;
            config.Slot0.kD = 0.0; // TODO: tune

            return config;
        }

        public static final double MIN_EXTENSION_METERS = 0.0;
        public static final double MIN_LENGTH_METERS = Units.inchesToMeters(28.566252);
        public static final double MAX_EXTENSION_METERS = Units.inchesToMeters(25);
        public static final double MAX_LENGTH_METERS = MIN_LENGTH_METERS + MAX_EXTENSION_METERS;

        public static final double MASS_KG = Units.lbsToKilograms(10.5082601) + LauncherConstants.MASS_KG;

        public static final double STATIC_STAGE_LENGTH_METERS = Units.inchesToMeters(26.066252);
        public static final double CARRIAGE_LENGTH_METERS = Units.inchesToMeters(31.312500);
        public static final double CARRIAGE_MASS_KG = Units.lbsToKilograms(1.8563182);
        public static final double STATIC_STAGE_MASS_KG = MASS_KG - CARRIAGE_MASS_KG;

        public static final double CARRIAGE_DRUM_RADIUS_METERS = (ROTOR_CIRCUMF_METERS / Math.PI) / 2;


        public static final double STATIC_STAGE_MOI = (1./3.) * STATIC_STAGE_MASS_KG * Math.pow(STATIC_STAGE_LENGTH_METERS/2, 2);
        public static final double CARRIAGE_BASE_MOI = (1./12.) * CARRIAGE_MASS_KG * Math.pow(CARRIAGE_LENGTH_METERS/2, 2);

    }

}
