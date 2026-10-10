package first.robot;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.opmode.Utility;

import first.robot.commands.StateMachineManager;
import first.robot.subsystems.launcher.LauncherConstants.LauncherStates;

public class OpModeContainer {
    
    @Teleop
    public static class Comp implements OpMode {

        private final Command TeleopSM;

        public Comp(Robot r) {
            TeleopSM = r.SMManager.teleop();

            r.driver.menu().onTrue(Command.requiring(r.drive).executing(co -> {
                r.drive.setPose(new Pose2d(r.drive.getPose().getTranslation(), new Rotation2d()));
            }).named("RESET HEADING"));

            r.operator.y().onTrue(r.launcher.setScoringState(LauncherStates.SELF_DIRECTING));
            r.operator.x().onTrue(r.launcher.setScoringState(LauncherStates.MANUAL));

            // adjustments
            Trigger operatorLeftYUp = new Trigger(() -> r.operator.getLeftY() < -0.9);
            operatorLeftYUp.whileTrue(r.telescope.adjustAngleDeg(10 * r.getPeriod()));
            Trigger operatorLeftYDown = new Trigger(() -> r.operator.getLeftY() > 0.9);
            operatorLeftYDown.whileTrue(r.telescope.adjustAngleDeg(-10 * r.getPeriod()));
            Trigger operatorRightYUp = new Trigger(() -> r.operator.getRightY() < -0.9);
            operatorRightYUp.whileTrue(r.telescope.adjustExtensionIn(1.5 * r.getPeriod()));
            Trigger operatorRightYDown = new Trigger(() -> r.operator.getRightY() > 0.9);
            operatorRightYDown.whileTrue(r.telescope.adjustExtensionIn(-1.5 * r.getPeriod()));

            r.operator.dpadUp().whileTrue(r.endEffector.adjustAngleDeg(10 * r.getPeriod()));
            r.operator.dpadDown().whileTrue(r.endEffector.adjustAngleDeg(-10 * r.getPeriod()));
            r.operator.dpadRight().whileTrue(r.endEffector.adjustVoltage(0.5 * r.getPeriod()));
            r.operator.dpadLeft().whileTrue(r.endEffector.adjustVoltage(-0.5 * r.getPeriod()));

            r.operator.rightBumper().whileTrue(r.launcher.adjustRPS(1 * r.getPeriod()));
            r.operator.leftBumper().whileTrue(r.launcher.adjustRPS(-1 * r.getPeriod()));

            // schedules the state machine to start as soon as op mode is selected
            // if the state machine exits (CLUMB state) you can press xbox to reinitiate it again
            Scheduler.getDefault().schedule(TeleopSM);
            r.driver.xbox()
                    .and(() -> !Scheduler.getDefault().isRunning(TeleopSM))
                    .onTrue(TeleopSM);
        }

        // @Override public void disabledPeriodic() {}
        // @Override public void start() {}
        // @Override public void periodic() {}
        // @Override public void end() {}
        // @Override public void close() {}
    }

    public static class Auto implements OpMode {
        public Auto(Command c) {
            Scheduler.getDefault().schedule(c);
        }
    }

    public static OpMode generateAuto(Command autoCommand) {
        return new Auto(autoCommand);
    }

    @Utility
    public static class Functional implements OpMode {
        private final Command functionalSM;

        public Functional(Robot r) {
            functionalSM = r.SMManager.functional();
            r.driver.xbox()
                    .and(() -> !Scheduler.getDefault().isRunning(functionalSM))
                    .onTrue(functionalSM);

            Scheduler.getDefault().schedule(functionalSM);
        }

    }

    @Utility
    public static class Tuning implements OpMode {
        public Tuning(Robot r) {
            Scheduler.getDefault().schedule(r.SMManager.tuning());
        }
    }

}
