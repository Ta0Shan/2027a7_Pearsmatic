// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.commands;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;

import first.robot.Constants;
import first.robot.Constants.CrystalColor;
import first.robot.Constants.FieldConstants;
import first.robot.Constants.SuperstructureStates;
import first.robot.Constants.FieldConstants.BlueFieldConstants;
import first.robot.Constants.FieldConstants.RedFieldConstants;
import first.robot.subsystems.drive.Drive;

import static org.wpilib.units.Units.Seconds;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

    private final Drive drive;
    private final Supplier<Pose2d> drivePose;

    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 7.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = Units.degreesToRadians(360);
    private static final double ANGLE_MAX_ACCELERATION = Units.degreesToRadians(720);

    private static final double DRIVE_kP = 7.0;
    private static final double DRIVE_kD = 0.4;
    private static final double DRIVE_MAX_VELOCITY = 3.0; // m/s
    private static final double DRIVE_MAX_ACCELERATION = 10.0; // m/s/s

    // Characterization has been commented because sim is ideal and ideally everything works
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    public DriveCommands(Drive drive) {
        this.drive = drive;
        drivePose = () -> this.drive.getPose();
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public Command joystickDrive(
                DoubleSupplier xSupplier,
                DoubleSupplier ySupplier,
                DoubleSupplier omegaSupplier) {
        return drive.run(co -> {
            while(true) {
                // Get linear velocity
                Translation2d linearVelocity =
                getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                // Apply rotation deadband
                double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                // Square rotation value for more precise control
                omega = Math.copySign(omega * omega, omega);
                // Convert to field relative speeds & send command
                ChassisVelocities velocities =
                    new ChassisVelocities(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega * drive.getMaxAngularSpeedRadPerSec());
                boolean isFlipped =
                    DriverStationBackend.getAlliance().isPresent()
                    && DriverStationBackend.getAlliance().get() == Alliance.RED;
                drive.runVelocity(
                    velocities.toRobotRelative(
                        isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()
                    )
                );
                co.yield();
            }
        }).named("JOYSTICK DRIVE");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public Command joystickDriveAtAngle(
                DoubleSupplier xSupplier,
                DoubleSupplier ySupplier,
                Supplier<Rotation2d> rotationSupplier) {

        // Construct command
        return drive.run(co -> {
            // Create PID controller
            ProfiledPIDController angleController =
                new ProfiledPIDController(
                    ANGLE_KP,
                    0.0,
                    ANGLE_KD,
                    new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
            angleController.enableContinuousInput(-Math.PI, Math.PI);

            // Reset PID controller when command starts
            angleController.reset(drive.getRotation().getRadians());
            while(true) {

                // Get linear velocity
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

                // Calculate angular speed
                double omega =
                    angleController.calculate(
                    drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

                // Convert to field relative speeds & send command
                ChassisVelocities velocities =
                    new ChassisVelocities(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
                boolean isFlipped =
                    DriverStationBackend.getAlliance().isPresent()
                    && DriverStationBackend.getAlliance().get() == Alliance.RED;
                drive.runVelocity(
                    velocities.toRobotRelative(
                        isFlipped
                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                        : drive.getRotation()
                    )
                );
                co.yield();
            }
        }).named("JOYSTICK DRIVE AT ANGLE " + Math.round(rotationSupplier.get().getDegrees() * 100.0) / 100.0 + "°");
    }

    public Command goToPose(Supplier<Pose2d> pose) {
        return drive.run(co -> {

            Pose2d goal = pose.get();

            Logger.recordOutput("Align/Travel Trajectory", new Translation2d[] {drive.getPose().getTranslation(), goal.getTranslation()});

            // Create PID controller
            ProfiledPIDController angleController =
                new ProfiledPIDController(
                    ANGLE_KP,
                    0.0,
                    ANGLE_KD,
                    new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
            angleController.enableContinuousInput(-Math.PI, Math.PI);
            angleController.setTolerance(Units.degreesToRadians(1.));

            // Reset PID controller when command starts
            angleController.reset(drive.getRotation().getRadians());

            ProfiledPIDController driveController =
                new ProfiledPIDController(
                    DRIVE_kP,
                    0.0,
                    DRIVE_kD,
                    new TrapezoidProfile.Constraints(DRIVE_MAX_VELOCITY, DRIVE_MAX_ACCELERATION));
            
            Translation2d error = goal.minus(drive.getPose()).getTranslation();
            Rotation2d direction = error.getAngle();

            ChassisVelocities currentVelocity = drive.getChassisVelocities();
            double velocityTowardsTarget = (currentVelocity.vx * direction.getCos()) + (currentVelocity.vy * direction.getSin());
            
            driveController.reset(error.getNorm(), velocityTowardsTarget); // "current position"
            // we set the position this way because the goal (pose) is thus (0, 0): that way, the drive pose IS the error
            driveController.setTolerance(Units.inchesToMeters(1));

            while(!driveController.atGoal() || !angleController.atGoal()) {

                error = goal.minus(drive.getPose()).getTranslation();
                direction = error.getAngle().plus(Rotation2d.k180deg);
                // flipped because the direction of the error vector is opposite the direction of the necessary robot velocity vector

                double twist = 
                    angleController.calculate(
                        drive.getRotation().getRadians(),
                        goal.getRotation().getRadians());
                
                double velocity = 
                    driveController.calculate(
                        error.getNorm(), // negative because this is the magnitude
                        0); // because the goal is being treated as (0, 0)
                
                Translation2d throttle = new Translation2d(
                    velocity,
                    direction);

                drive.runVelocity(new ChassisVelocities(throttle.getX(), throttle.getY(), twist));
                
                Logger.recordOutput("Align/Goal Pose", goal);
                Logger.recordOutput("Align/Translation Error", error.getNorm());
                Logger.recordOutput("Align/Rotation Error", goal.getRotation().minus(drive.getRotation()).getDegrees());
                Logger.recordOutput("Align/Throttle", velocity);
                Logger.recordOutput("Align/Direction", direction.getDegrees());
                Logger.recordOutput("Align/Twist", twist);

                co.yield();
            }

        }).named(String.format("GO TO (%.2f, %.2f) AT %.2f°", pose.get().getX(), pose.get().getY(), pose.get().getRotation().getDegrees()));
    }

    public Command shuttleAlign() {
        return drive.run(co -> {
            Translation2d currentPose = drive.getPose().getTranslation();
            Translation2d targetPose = 
                DriverStationBackend.getAlliance().orElse(Alliance.RED) == Alliance.RED
                ? RedFieldConstants.CLASSIFIER_AIM_TARGET
                : BlueFieldConstants.CLASSIFIER_AIM_TARGET;
            
            Rotation2d targetRotation = targetPose.minus(currentPose).getAngle().minus(Rotation2d.k180deg);
            
            co.await(goToPose(() -> new Pose2d(drive.getPose().getTranslation(), targetRotation)));
        }).named("SHUTTLE ALIGN");
    }

    public Command neutralAlign(Supplier<SuperstructureStates> state) {
        return drive.run(co -> {
            boolean isL1 = state.get() == SuperstructureStates.L1_BACK || state.get() == SuperstructureStates.L1_FRONT;
            boolean isRed = DriverStationBackend.getAlliance().orElse(Alliance.RED) == Alliance.RED;

            Pose2d closestPose = drive.getPose().nearest(Arrays.asList(
                isRed ?
                  isL1 ? RedFieldConstants.LOWER_SHAFTS : RedFieldConstants.UPPER_SHAFTS
                : isL1 ? BlueFieldConstants.LOWER_SHAFTS : BlueFieldConstants.UPPER_SHAFTS
            )).plus(isL1 ? FieldConstants.LSHAFT_GOAL_DIST : FieldConstants.USHAFT_GOAL_DIST)
            ;
            co.await(goToPose(() -> closestPose));
        }).named("NEUTAL ALIGN");
    }

    public Command coloredAlign(Supplier<SuperstructureStates> state, Supplier<CrystalColor> crystalColor) {
        return drive.run(co -> {
            boolean isL1 = state.get() == SuperstructureStates.L1_BACK || state.get() == SuperstructureStates.L1_FRONT;
            boolean isRed = DriverStationBackend.getAlliance().orElse(Alliance.RED) == Alliance.RED;

            Pose2d[] shaftList = isRed ?
                  isL1 ? RedFieldConstants.LOWER_SHAFTS : RedFieldConstants.UPPER_SHAFTS
                : isL1 ? BlueFieldConstants.LOWER_SHAFTS : BlueFieldConstants.UPPER_SHAFTS
            ;

            Pose2d[] validShafts = crystalColor.get()!=CrystalColor.NONE ?
                  FieldConstants.getValidShaft(isL1, shaftList, crystalColor.get())
                : shaftList
            ;

            Pose2d closestPose = drive.getPose().nearest(Arrays.asList(validShafts))
                .plus(isL1 ? FieldConstants.LSHAFT_GOAL_DIST : FieldConstants.USHAFT_GOAL_DIST);
            co.await(goToPose(() -> closestPose));
        }).named("");
    }

    public Command driveCircle() {
        return drive.run(co -> {
            Rotation2d direction = Rotation2d.fromDegrees(1);
            Translation2d linearVelocity = new Translation2d(0.5, direction);

            while(!direction.equals(Rotation2d.kZero)) {
                ChassisVelocities velocity = new ChassisVelocities(
                    linearVelocity.getX(),
                    linearVelocity.getY(),
                    0.0);
                
                drive.runVelocity(velocity);

                direction = direction.plus(Rotation2d.fromDegrees(0.5));
                linearVelocity = new Translation2d(0.5, direction);
                co.yield();
            }
        }).named("DRIVE CIRCLE");
    }

    public Command spin(double time) {
        return drive.run(co -> {
            drive.runVelocity(new ChassisVelocities(0, 0, 1));
            co.wait(Seconds.of(time));
        }).named("SPIN " + time + "s");
    }






    // CHARACTERIZATION BELOW =====================================================================





    private List<Double> velocitySamples = new LinkedList<>();
    private List<Double> voltageSamples = new LinkedList<>();

    /**
    * Measures the velocity feedforward constants for the drive motors.
    *
    * <p>This command should only be used in voltage control mode.
    */
    public Command feedforwardCharacterization() {
        return drive.run(co -> {
            Timer timer = new Timer();

            velocitySamples.clear();
            voltageSamples.clear();

            // Allow modules to orient
            drive.runCharacterization(0.0);

            // Start timer
            timer.restart();        

            // Accelerate and gather data
            while(true) {
                double voltage = timer.get() * FF_RAMP_RATE;
                drive.runCharacterization(voltage);
                velocitySamples.add(drive.getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
                co.yield();
            }

        }).whenCanceled(() -> {
            // When cancelled, calculate and print results
            int n = velocitySamples.size();
            double sumX = 0.0;
            double sumY = 0.0;
            double sumXY = 0.0;
            double sumX2 = 0.0;
            for (int i = 0; i < n; i++) {
            sumX += velocitySamples.get(i);
            sumY += voltageSamples.get(i);
            sumXY += velocitySamples.get(i) * voltageSamples.get(i);
            sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
            }
            double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
            double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

            NumberFormat formatter = new DecimalFormat("#0.00000");
            System.out.println("********** Drive FF Characterization Results **********");
            System.out.println("\tkS: " + formatter.format(kS));
            System.out.println("\tkV: " + formatter.format(kV));
        })
        .named("FF CHARACTERIZATION");
    }

    private WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

/** Measures the robot's wheel radius by spinning in a circle. */
    public Command wheelRadiusCharacterization() {
        return drive.run(co -> {
            SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
            state = new WheelRadiusCharacterizationState();

            co.await(
                Command.parallel(
                    // Drive control command
                    drive.run(co2 -> {
                        // Reset acceleration limiter
                        limiter.reset(0.0);

                        // Turn in place, accelerating up to full speed
                        while(true) {
                            double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                            drive.runVelocity(new ChassisVelocities(0, 0, speed));
                            co2.yield();
                        }
                    }).named("TURN IN PLACE"),

                    // Measurement command
                    Command.noRequirements(co2 -> {
                        // Wait for modules to fully orient before starting measurement
                        co2.wait(Seconds.of(1.));

                        // Record starting measurement
                        state.positions = drive.getWheelRadiusCharacterizationPositions();
                        state.lastAngle = drive.getRotation();
                        state.gyroDelta = 0.0;

                        // Update gyro delta
                        while(true) {
                            Rotation2d rotation = drive.getRotation();
                            state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                            state.lastAngle = rotation;
                        }
                    }).named("MEASURE ROTATION")
                ).named("TURN IN PLACE")
            );
        }).whenCanceled(() -> {
            // When cancelled, calculate and print results
            double[] positions = drive.getWheelRadiusCharacterizationPositions();
            double wheelDelta = 0.0;
            for (int i = 0; i < 4; i++) {
                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
            }
            double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

            NumberFormat formatter = new DecimalFormat("#0.000");
            System.out.println(
                "********** Wheel Radius Characterization Results **********");
            System.out.println(
                "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
            System.out.println(
                "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
            System.out.println(
                "\tWheel Radius: "
                + formatter.format(wheelRadius)
                + " meters, "
                + formatter.format(Units.metersToInches(wheelRadius))
                + " inches");
        })
        .named("WHEEL RADIUS CHARACTERIZATION");
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }
}
