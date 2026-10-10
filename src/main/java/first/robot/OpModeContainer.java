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

        private final CommandXboxController driver;
        private final CommandXboxController operator;

        public Comp(Robot r) {
            TeleopSM = r.SMManager.teleop();
            driver = r.driver;
            operator = r.operator;

            driver.menu().onTrue(Command.requiring(r.drive).executing(co -> {
                r.drive.setPose(new Pose2d(r.drive.getPose().getTranslation(), new Rotation2d()));
            }).named("RESET HEADING"));

            operator.y().onTrue(r.launcher.setScoringState(LauncherStates.SELF_DIRECTING));
            operator.x().onTrue(r.launcher.setScoringState(LauncherStates.MANUAL));

            // adjustments
            Trigger operatorLeftYUp = new Trigger(() -> operator.getLeftY() < -0.9);
            operatorLeftYUp.whileTrue(r.telescope.adjustAngleDeg(10 * r.getPeriod()));
            Trigger operatorLeftYDown = new Trigger(() -> operator.getLeftY() > 0.9);
            operatorLeftYDown.whileTrue(r.telescope.adjustAngleDeg(-10 * r.getPeriod()));
            Trigger operatorRightYUp = new Trigger(() -> operator.getRightY() < -0.9);
            operatorRightYUp.whileTrue(r.telescope.adjustExtensionIn(1.5 * r.getPeriod()));
            Trigger operatorRightYDown = new Trigger(() -> operator.getRightY() > 0.9);
            operatorRightYDown.whileTrue(r.telescope.adjustExtensionIn(-1.5 * r.getPeriod()));

            operator.dpadUp().whileTrue(r.endEffector.adjustAngleDeg(10 * r.getPeriod()));
            operator.dpadDown().whileTrue(r.endEffector.adjustAngleDeg(-10 * r.getPeriod()));
            operator.dpadRight().whileTrue(r.endEffector.adjustVoltage(0.5 * r.getPeriod()));
            operator.dpadLeft().whileTrue(r.endEffector.adjustVoltage(-0.5 * r.getPeriod()));

            operator.rightBumper().whileTrue(r.launcher.adjustRPS(1 * r.getPeriod()));
            operator.leftBumper().whileTrue(r.launcher.adjustRPS(-1 * r.getPeriod()));
        }

        // @Override
        // public void disabledPeriodic() {}

        @Override
        public void start() {
            if (!Scheduler.getDefault().isRunning(TeleopSM)) Scheduler.getDefault().schedule(TeleopSM);
        }

        // @Override
        // public void periodic() {}

        // @Override
        // public void end() {}

        // @Override
        // public void close() {}
    }

    public static class Auto implements OpMode {

        private final Command autoCommand;

        public Auto(Command c) {
            autoCommand = c;
        }

        @Override public void start() {Scheduler.getDefault().schedule(autoCommand);}
        @Override public void close() {Scheduler.getDefault().cancel(autoCommand);}
    }

    public static OpMode generateAuto(Command autoCommand) {
        return new Auto(autoCommand);
    }

    @Utility
    public static class Functional implements OpMode {
        private final Command functionalSM;

        public Functional(Robot r) {
            functionalSM = r.SMManager.functional();
        }

        @Override
        public void start() {Scheduler.getDefault().schedule(functionalSM);}

        @Override
        public void close() {Scheduler.getDefault().cancel(functionalSM);}
    }

    @Utility
    public static class Tuning implements OpMode {
        private final Command tuningSM;

        public Tuning(Robot r) {
            tuningSM = r.SMManager.tuning();
        }

        @Override
        public void start() {Scheduler.getDefault().schedule(tuningSM);}

        @Override
        public void close() {Scheduler.getDefault().cancel(tuningSM);}
    }

}
