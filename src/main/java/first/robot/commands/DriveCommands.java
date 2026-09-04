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
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;

import first.robot.Constants.CrystalColor;
import first.robot.Constants.FieldConstants;
import first.robot.Constants.SuperstructureStates;
import first.robot.Constants.FieldConstants.BlueFieldConstants;
import first.robot.Constants.FieldConstants.RedFieldConstants;
import first.robot.subsystems.drive.Drive;
import first.robot.util.LoggedTunableNumber;

import static org.wpilib.units.Units.Seconds;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class DriveCommands {

    private final Drive drive;

    private static final double DEADBAND = 0.1;
    // private static final double ANGLE_KP = 7.0;
    // private static final double ANGLE_KD = 0.4;
    // private static final double ANGLE_MAX_VELOCITY = Units.degreesToRadians(360);
    // private static final double ANGLE_MAX_ACCELERATION = Units.degreesToRadians(720);
    private final LoggedTunableNumber anglekP = new LoggedTunableNumber("Align/Angle/kP", 7.0);
    private final LoggedTunableNumber anglekD = new LoggedTunableNumber("Align/Angle/kD", 0.4);
    private final LoggedTunableNumber angleMaxVel = new LoggedTunableNumber("Align/Angle/Max Velocity Deg", 360.0);
    private final LoggedTunableNumber angleMaxAccel = new LoggedTunableNumber("Align/Angle/Max Acceleration Deg", 720.0);
    
    // private static final double DRIVE_kP = 7.0;
    // private static final double DRIVE_kD = 0.4;
    // private static final double DRIVE_MAX_VELOCITY = 4.0; // m/s
    // private static final double DRIVE_MAX_ACCELERATION = 10.0; // m/s/s
    private final LoggedTunableNumber drivekP = new LoggedTunableNumber("Align/Drive/kP", 7.0);
    private final LoggedTunableNumber drivekD = new LoggedTunableNumber("Align/Drive/kD", 0.4);
    private final LoggedTunableNumber driveMaxVel = new LoggedTunableNumber("Align/Drive/Max Velocity m/s", 4.0);
    private final LoggedTunableNumber driveMaxAccel = new LoggedTunableNumber("Align/Drive/Max Acceleration m/s/s", 10.0);

    // Characterization has been commented because sim is ideal and ideally everything works
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    public DriveCommands(Drive drive) {
        this.drive = drive;
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.ZERO, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.ZERO))
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
                    anglekP.get(),
                    0.0,
                    anglekD.get(),
                    new TrapezoidProfile.Constraints(Units.degreesToRadians(angleMaxVel.get()), Units.degreesToRadians(angleMaxAccel.get())));
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

            Logger.recordOutput("Align/Trajectory", new Translation2d[] {drive.getPose().getTranslation(), goal.getTranslation()});

            // Create PID controller
            ProfiledPIDController angleController =
                new ProfiledPIDController(
                    anglekP.get(),
                    0.0,
                    anglekD.get(),
                    new TrapezoidProfile.Constraints(Units.degreesToRadians(angleMaxVel.get()), Units.degreesToRadians(angleMaxAccel.get())));
            angleController.enableContinuousInput(-Math.PI, Math.PI);
            angleController.setGoal(goal.getRotation().getRadians());
            angleController.setTolerance(Units.degreesToRadians(1.));

            // Reset PID controller when command starts
            angleController.reset(drive.getRotation().getRadians());

            ProfiledPIDController driveController =
                new ProfiledPIDController(
                    drivekP.get(),
                    0.0,
                    drivekD.get(),
                    new TrapezoidProfile.Constraints(driveMaxVel.get(), driveMaxAccel.get()));
            
            Translation2d error = goal.minus(drive.getPose()).getTranslation();
            Rotation2d direction = error.getAngle().get();

            ChassisVelocities currentVelocity = drive.getChassisVelocities();
            double velocityTowardsTarget = (currentVelocity.vx * direction.getCos()) + (currentVelocity.vy * direction.getSin());
            
            driveController.reset(error.getNorm(), velocityTowardsTarget); // "current position"
            // we set the position this way because the goal (pose) is thus (0, 0): that way, the drive pose IS the error
            driveController.setGoal(0);
            driveController.setTolerance(Units.inchesToMeters(1));

            while(!driveController.atGoal() || !angleController.atGoal()) {

                error = goal.minus(drive.getPose()).getTranslation();
                direction = error.getAngle().get().plus(Rotation2d.PI);
                // flipped because the direction of the error vector is opposite the direction of the necessary robot velocity vector

                double twist = 
                    angleController.calculate(drive.getRotation().getRadians());
                
                double velocity = 
                    driveController.calculate(
                        error.getNorm());
                
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
                
                Rotation2d targetRotation = targetPose.minus(currentPose).getAngle().get().minus(Rotation2d.PI);
                
                co.await(goToPose(() -> new Pose2d(drive.getPose().getTranslation(), targetRotation)));
            }).named("ALIGN SHUTTLE");
        }
        
    public double getDistanceFromClassifier() {
        Translation2d currentPose = drive.getPose().getTranslation();
        Translation2d targetPose = 
            DriverStationBackend.getAlliance().orElse(Alliance.RED) == Alliance.RED
            ? RedFieldConstants.CLASSIFIER_AIM_TARGET
            : BlueFieldConstants.CLASSIFIER_AIM_TARGET;

        return targetPose.minus(currentPose).getNorm();
    }

    public Command align(Supplier<SuperstructureStates> state, Supplier<CrystalColor> crystalColor) {
        return drive.run(co -> {
            if (state.get() == SuperstructureStates.CLASSIFIER_BACK || state.get() == SuperstructureStates.CLASSIFIER_FRONT) {
                co.await(classifierAlign(state));
            } else {
                boolean isL1 = state.get() == SuperstructureStates.L1_BACK || state.get() == SuperstructureStates.L1_FRONT;
                boolean isRed = DriverStationBackend.getAlliance().orElse(Alliance.RED) == Alliance.RED;
                boolean isFront = state.get() == SuperstructureStates.L1_FRONT || state.get() == SuperstructureStates.L2_FRONT;
                // boolean isFront = isFront(() -> DriverStationBackend.getAlliance(), state);

                Pose2d[] goalList = isRed
                    ? (isL1 ? RedFieldConstants.LOWER_SHAFT_FACES : RedFieldConstants.UPPER_SHAFT_VERTICES)
                    : (isL1 ? BlueFieldConstants.LOWER_SHAFT_FACES : BlueFieldConstants.UPPER_SHAFT_VERTICES);

                Pose2d[] validGoals = crystalColor.get() != CrystalColor.NONE ?
                    FieldConstants.getValidGoal(isL1, goalList, crystalColor.get())
                    : goalList
                    ;

                Pose2d targetPose = drive.getPose().nearest(Arrays.asList(validGoals))
                    .plus(isFront ? FieldConstants.ALIGN_OFFSET_SHORT : FieldConstants.ALIGN_OFFSET_LONG)
                    .plus(isFront ? new Transform2d() : new Transform2d(0, 0, Rotation2d.PI))
                    ;
                co.await(goToPose(() -> targetPose));
            }
        }).named("COLORED ALIGN " + (crystalColor.get() == CrystalColor.NONE ? "ANY" : crystalColor.get().name()));
    }

    public Command align(Supplier<SuperstructureStates> state) {
        return drive.run(co -> {
            if (state.get() == SuperstructureStates.CLASSIFIER_FRONT || state.get() == SuperstructureStates.CLASSIFIER_BACK) {
                co.await(classifierAlign(state));
            } else {
                co.await(align(state, () -> CrystalColor.NONE));
            }
        }).named("NEUTRAL ALIGN");
    }

    public Command classifierAlign(Supplier<SuperstructureStates> state) {
        return drive.run(co -> {
            boolean isRed = DriverStationBackend.getAlliance().orElse(Alliance.RED) == Alliance.RED;
            boolean isFront = state.get() == SuperstructureStates.CLASSIFIER_FRONT;
            // boolean isFront = isFront(() -> DriverStationBackend.getAlliance(), state);

            Pose2d targetPose = (isRed ?
                        RedFieldConstants.CLASSIFIER_CENTER
                        : BlueFieldConstants.CLASSIFIER_CENTER)
                        .plus(FieldConstants.ALIGN_OFFSET_SHORT)
                        .plus(isFront ? new Transform2d() : new Transform2d(0, 0, Rotation2d.PI));
            ;
            co.await(goToPose(() -> targetPose));
        }).named("CLASSIFIER ALIGN");
    }

    public boolean isFront(Supplier<SuperstructureStates> state) {
        boolean isRed = DriverStationBackend.getAlliance().orElse(Alliance.RED) == Alliance.RED;
        boolean isClassifier = state.get() == SuperstructureStates.CLASSIFIER_FRONT || state.get() == SuperstructureStates.CLASSIFIER_BACK;

        return isClassifier
            ? isRed
                ? Math.abs(drive.getRotation().minus(RedFieldConstants.CLASSIFIER_CENTER.getTranslation().minus(drive.getPose().getTranslation()).getAngle().get()).getDegrees()) <= 90
                : Math.abs(drive.getRotation().minus(BlueFieldConstants.CLASSIFIER_CENTER.getTranslation().minus(drive.getPose().getTranslation()).getAngle().get()).getDegrees()) <= 90
            : isRed
                ? Math.abs(drive.getRotation().minus(RedFieldConstants.CAVE_CENTER.minus(drive.getPose().getTranslation()).getAngle().get()).getDegrees()) <= 90
                : Math.abs(drive.getRotation().minus(BlueFieldConstants.CAVE_CENTER.minus(drive.getPose().getTranslation()).getAngle().get()).getDegrees()) <= 90;
        // if (isRed) { // if isRed
        //     if (isClassifier) {
        //         return Math.abs(drive.getRotation().minus(RedFieldConstants.CLASSIFIER_CENTER.getTranslation().minus(drive.getPose().getTranslation()).getAngle()).getDegrees()) <= 90;
        //     } // else isCave
        //     return Math.abs(drive.getRotation().minus(RedFieldConstants.CAVE_CENTER.minus(drive.getPose().getTranslation()).getAngle()).getDegrees()) <= 90;
        // } // else isBlue
        // if (isClassifier) {
        //     return Math.abs(drive.getRotation().minus(BlueFieldConstants.CLASSIFIER_CENTER.getTranslation().minus(drive.getPose().getTranslation()).getAngle()).getDegrees()) <= 90;
        // } // else isBlue && isCave
        // return Math.abs(drive.getRotation().minus(BlueFieldConstants.CAVE_CENTER.minus(drive.getPose().getTranslation()).getAngle()).getDegrees()) <= 90;
    }

    public Command driveCircle() {
        return drive.run(co -> {
            Rotation2d direction = Rotation2d.fromDegrees(1);
            Translation2d linearVelocity = new Translation2d(0.5, direction);

            while(!direction.equals(Rotation2d.ZERO)) {
                ChassisVelocities velocity = new ChassisVelocities(
                    linearVelocity.getX(),
                    linearVelocity.getY(),
                    0.0);
                
                drive.runVelocity(velocity);

                direction = direction.plus(Rotation2d.fromDegrees(0.5));
                linearVelocity = new Translation2d(0.5, direction);
                co.yield();
            }
            drive.runVelocity(new ChassisVelocities(0, 0, 0));
        }).named("DRIVE CIRCLE");
    }

    public Command spin(double time) {
        return drive.run(co -> {
            drive.runVelocity(new ChassisVelocities(0, 0, 1));
            co.wait(Seconds.of(time));
            drive.runVelocity(new ChassisVelocities(0, 0, 0));
        }).named("SPIN " + time + "s");
    }






    // CHARACTERIZATION BELOW =====================================================================





    private ArrayList<Double> velocitySamples = new ArrayList<Double>();
    private ArrayList<Double> voltageSamples = new ArrayList<Double>();

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
            co.wait(Seconds.of(FF_START_DELAY));

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
                            co2.yield();
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
        Rotation2d lastAngle = Rotation2d.ZERO;
        double gyroDelta = 0.0;
    }
}
