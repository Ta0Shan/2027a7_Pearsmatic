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

import first.robot.commands.StateMachineManager;
import first.robot.subsystems.launcher.LauncherConstants.LauncherStates;

public class OpModeContainer {
    
    @Teleop
    public static class Comp implements OpMode {

        private final StateMachineManager SMManager;

        private final CommandXboxController driver = new CommandXboxController(0);
        private final CommandXboxController operator = new CommandXboxController(1);

        public Comp(Robot r) {
            SMManager = new StateMachineManager(
                r.telescope,
                r.launcher,
                r.endEffector,
                r.drive,

                () -> driver.getLeftX(),
                () -> driver.getLeftY(),
                () -> driver.getRightX(),
                driver.a(),
                driver.leftBumper(),
                driver.leftTrigger(0.8),
                driver.x(),
                driver.y(),
                driver.b(),
                driver.dpadUp(),
                driver.rightBumper(),
                driver.rightTrigger(0.8)
            );

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
        public void start() {Scheduler.getDefault().schedule(SMManager.teleop());}

        // @Override
        // public void periodic() {}

        // @Override
        // public void end() {}

        // @Override
        // public void close() {}
    }

    public static OpMode generateAuto(Robot r, Command autoCommand) {
        @Autonomous
        class Auto implements OpMode {
            private final Command autoCommand;
            public Auto(Command autoCommand) {this.autoCommand = autoCommand;}
            @Override public void start() {Scheduler.getDefault().schedule(autoCommand);}
        }
        return new Auto(autoCommand);
    }

}
