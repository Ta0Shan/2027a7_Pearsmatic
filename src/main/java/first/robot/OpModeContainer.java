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
        }

        // @Override public void disabledPeriodic() {}
        @Override public void start() {
            Scheduler.getDefault().schedule(TeleopSM);
            // TODO: figure out why this one runs while disabled? probably bc it PARKs a command?
            // it doesn't mess up functionality because one state machine ends up cancelling the rest and so when an input happens there's only one response but still
        }
        // @Override public void periodic() {}
        // @Override public void end() {}
        // @Override public void close() {}
    }

    public static class Auto implements OpMode {
        private final Command auto;
        public Auto(Command c) {
            auto = c;
        }

        @Override public void start() {Scheduler.getDefault().schedule(auto);}
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

        @Override public void start() {Scheduler.getDefault().schedule(functionalSM);}

    }

    @Utility
    public static class Tuning implements OpMode {
        private final Command tuningSM;

        public Tuning(Robot r) {
            tuningSM = r.SMManager.tuning();
        }

        @Override public void start() {Scheduler.getDefault().schedule(tuningSM);}
    }

    @Utility
    public static class CharacterizeFF implements OpMode {
        private final Command characterization;
        
        public CharacterizeFF(Robot r) {
            characterization = r.SMManager.ffCharacterization();
        }

        @Override public void start() {Scheduler.getDefault().schedule(characterization);}
        
    }

    @Utility
    public static class CharacterizeWheels implements OpMode {
        private final Command characterization;
        
        public CharacterizeWheels(Robot r) {
            characterization = r.SMManager.wheelRadiusCharacterization();
        }

        @Override public void start() {Scheduler.getDefault().schedule(characterization);}
    }

}
