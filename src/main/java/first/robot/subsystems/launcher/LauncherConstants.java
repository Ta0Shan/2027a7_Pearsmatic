package first.robot.subsystems.launcher;

import org.wpilib.math.util.Units;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.robot.Constants;

public class LauncherConstants {

    public static enum LauncherStates {
        OFF,
        SELF_DIRECTING,
        MANUAL,
        TUNING
        ;
    }

    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;

    public static final int LAUNCHER_1_ID = 41;
    public static final int LAUNCHER_2_ID = 42;

    public static final double REDUCTION = (32.0 / 22.0);

    public static final double RPS_DIFFERENCE = 8.0;

    public static final TalonFXConfiguration CONFIG(boolean isMotor1) {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        if (isMotor1) {
            config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        } else {
            config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        // config.Slot0.kV = 0.12;
        config.Slot0.kP = 5.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0; // TODO: tune

        return config;
    }

    public static final double FLYWHEEL_CIRCUMF_METERS = Units.inchesToMeters(Math.PI * 4.0);

    public static final double MOTOR_MAX_SPEED_RPS = 95;
    public static final double FLYWHEEL_MAX_SPEED_RPS = MOTOR_MAX_SPEED_RPS/REDUCTION;

    public static final double MASS_KG = Units.lbsToKilograms(12.2648713);

    // excuse the jank constant its for sim
    public static final double FLYWHEEL_DIST_FROM_STATIC_STAGE_METERS = Units.inchesToMeters(10.985093);
}
