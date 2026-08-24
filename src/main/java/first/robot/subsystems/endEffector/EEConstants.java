package first.robot.subsystems.endEffector;

import org.wpilib.hardware.bus.I2C.Port;
import org.wpilib.math.util.Units;
import org.wpilib.util.Color;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import first.robot.Constants;

public class EEConstants {

    public static enum WristStates {
        STOWED(0),
        DEPLOYED(201),
        L1_FRONT(166),
        L1_BACK(43),
        L2_FRONT(166),
        L2_BACK(59),
        CLASSIFIER_FRONT(201),
        CLASSIFIER_BACK(49),
        TUNING(0)
        ;

        public final double angleDeg;

        private WristStates(double angleFromStowDeg) {
            this.angleDeg = angleFromStowDeg;
        }
    }

    public static enum RollerStates {
        IDLE(0),
        FWD(5),
        REV(-5),
        FAST_FWD(10),
        FAST_REV(-10),
        TUNING(0)
        ;

        public final double voltage;

        private RollerStates(double voltage) {
            this.voltage = voltage;
        }

    }

    public static final CANBus CAN_BUS = Constants.SUPERSTRUCTURE_CAN_BUS;

    public static final int WRIST_ID = 51;
    public static final int ROLLERS_ID = 52;

    public static final double MIN_ANGLE_DEG = 0.0;
    public static final double MAX_ANGLE_DEG = 201.0;

    public static final Port COLOR_SENSOR_PORT = Port.PORT_0;
    public static final Color GREEN = new Color(0, 255, 0);    // #00ff00
    public static final Color YELLOW = new Color(255, 255, 0); // #ffff00
    public static final Color ORANGE = new Color(255, 165, 0); // #ffa500
    public static final Color PURPLE = new Color(128, 0, 128); // #800080

    public static final Color[] CRYSTAL_COLORS = {GREEN, YELLOW, ORANGE, PURPLE};

    public static final double WRIST_REDUCTION = (32.0/12.0) * (32.0/12.0) * (50.0/12.0); // (36.0 / 8.0) * (38.0/14.0) * (36.0/14.0) * (20.0/16.0);
    public static final double ROLLER_REDUCTION = (22.0/22.0) * (36.0/16.0);

    public static final double ROLLER_CIRCUMF_METERS = Units.inchesToMeters(Math.PI * 2.0);

    public static final TalonFXConfiguration WRIST_CONFIG() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 1.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0; // TODO: tune

        return config;
    }

    public static final TalonFXConfiguration ROLLER_CONFIG() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 60;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 60;

        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = 0.0;
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0; // TODO: tune if you want to use VelocityVoltage

        return config;
    }
    public static final double LENGTH_METERS = Units.inchesToMeters(8); // TODO
    public static final double MASS_KG = Units.lbsToKilograms(10); // some parts don't have defined weight, +0.3 lbs to be safe

    // excuse the jank constant its for sim
    public static final double STARTING_ANGLE_OFFSET_FROM_PARALLEL_DEG = 146.0;

}
