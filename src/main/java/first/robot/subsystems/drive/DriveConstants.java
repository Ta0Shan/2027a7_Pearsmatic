package first.robot.subsystems.drive;

import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Volts;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Velocity;
import org.wpilib.units.measure.Voltage;

import first.robot.generated.TunerConstants;

public class DriveConstants {

    public static final Velocity<VoltageUnit> SYSID_QUASI_RAMP_RATE = Volts.of(0.1).per(Second);
    public static final Voltage SYSID_DYNAM_STEP_VOLTAGE = Volts.of(7);
    public static final Time SYSID_SAFETY_TIMEOUT = Second.of(10);

    // TunerConstants doesn't include these constants, so they are declared locally
    public static final double ODOMETRY_FREQUENCY = TunerConstants.kCANBus.isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS =
        Math.max(
            Math.max(
                Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
                Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
            Math.max(
                Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
                Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

    
    // TunerConstants doesn't support separate sim constants, so they are declared locally
    public static final double SIM_DRIVE_KP = 0.05;
    public static final double SIM_DRIVE_KD = 0.0;
    public static final double SIM_DRIVE_KS = 0.0;
    public static final double SIM_DRIVE_KV_ROT =
        0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    public static final double SIM_DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / SIM_DRIVE_KV_ROT);
    public static final double SIM_TURN_KP = 8.0;
    public static final double SIM_TURN_KD = 0.0;
    public static final DCMotor SIM_DRIVE_GEARBOX = DCMotor.getKrakenX60Foc(1);
    public static final DCMotor SIM_TURN_GEARBOX = DCMotor.getKrakenX60Foc(1);

    public static record GyroInputs(
        boolean isConnected,
        Rotation2d yawPosition,
        double yawVelocityRadPerSec,
        double[] odometryYawTimestamps,
        Rotation2d[] odometryYawPositions
    ) {
        public GyroInputs() {
            this(
                false,
                Rotation2d.ZERO,
                0.0,
                new double[] {},
                new Rotation2d[] {}
            );
        }
    }

    public static record ModuleInputs(
        boolean driveConnected,
        double drivePositionRad,
        double driveVelocityRadPerSec,
        double driveAppliedVolts,
        double driveCurrentAmps,

        boolean turnConnected,
        boolean turnEncoderConnected,
        Rotation2d turnAbsolutePosition,
        Rotation2d turnPosition,
        double turnVelocityRadPerSec,
        double turnAppliedVolts,
        double turnCurrentAmps,

        double[] odometryTimestamps,
        double[] odometryDrivePositionsRad,
        Rotation2d[] odometryTurnPositions
    ) {
        public ModuleInputs() {
            this(
                false,
                0.0,
                0.0,
                0.0,
                0.0,

                false,
                false,
                Rotation2d.ZERO,
                Rotation2d.ZERO,
                0.0,
                0.0,
                0.0,

                new double[] {},
                new double[] {},
                new Rotation2d[] {}
            );
        }
    }
}
