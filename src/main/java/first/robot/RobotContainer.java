// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;

import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.NiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.smartdashboard.SmartDashboard;

import first.robot.Constants.CrystalColor;
import first.robot.Constants.Mode;
import first.robot.Constants.SuperstructureStates;
import first.robot.commands.SuperstructureCommands;
import first.robot.commands.DriveCommands;
import first.robot.commands.StateMachineManager;
import first.robot.generated.TunerConstants;
import first.robot.subsystems.MechVisualizer;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.drive.GyroIO;
import first.robot.subsystems.drive.GyroIOPigeon2;
import first.robot.subsystems.drive.ModuleIO;
import first.robot.subsystems.drive.ModuleIOSim;
import first.robot.subsystems.drive.ModuleIOTalonFX;
import first.robot.subsystems.endEffector.EE;
import first.robot.subsystems.endEffector.EEIO;
import first.robot.subsystems.endEffector.EEIOReal;
import first.robot.subsystems.endEffector.EEIOSim;
import first.robot.subsystems.launcher.Launcher;
import first.robot.subsystems.launcher.LauncherIO;
import first.robot.subsystems.launcher.LauncherIOReal;
import first.robot.subsystems.launcher.LauncherIOSim;
import first.robot.subsystems.telescope.Telescope;
import first.robot.subsystems.telescope.TelescopeIO;
import first.robot.subsystems.telescope.TelescopeIOReal;
import first.robot.subsystems.telescope.TelescopeIOSim;
import first.robot.subsystems.vision.Vision;
import first.robot.util.PhoenixUtil;

public class RobotContainer {

  public final CommandNiDsXboxController driver;
  public final CommandNiDsXboxController operator;
  private ArrayList<Trigger> boundTriggers;

  private final SendableChooser<Command> autoChooser;

  private final Drive drive;

  private final Telescope telescope;
  private final Launcher launcher;
  private final EE endEffector;

  // private final Vision vision;

  private final StateMachineManager SMManager;

  private final MechVisualizer visualizer2d;

  public RobotContainer() {
    driver = new CommandNiDsXboxController(0);
    operator = new CommandNiDsXboxController(1);
    boundTriggers = new ArrayList<Trigger>();

    autoChooser = new SendableChooser<>();

    switch(Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight)
        );
        telescope = new Telescope(new TelescopeIOReal());
        endEffector = new EE(new EEIOReal());
        launcher = new Launcher(new LauncherIOReal());
        break;
      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        
        drive = new Drive(
          new GyroIO() {},
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight)
        );
        telescope = new Telescope(new TelescopeIOSim());
        endEffector = new EE(new EEIOSim());
        launcher = new Launcher(new LauncherIOSim());
        break;
      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {}
        );
        telescope = new Telescope(new TelescopeIO() {});
        endEffector = new EE(new EEIO() {});
        launcher = new Launcher(new LauncherIO() {});
        break;
    }


    // configuring state machine
    SMManager = new StateMachineManager(
      // superstructureCommands,
      // driveCommands,
      telescope,
      launcher,
      endEffector,
      drive,
      () -> -driver.getLeftX(),
      () -> driver.getLeftY(),
      () -> -driver.getRightX(),
      driver.b(),
      driver.leftBumper(),
      driver.leftTrigger(0.9),
      driver.x(),
      driver.y(),
      driver.a(),
      driver.povUp(),
      driver.rightBumper(),
      driver.rightTrigger(0.9)
    );
  
    visualizer2d = new MechVisualizer();

    setUpAutonomousCommand();
  }

  public void setUpAutonomousCommand() {
    // autoChooser.addOption("Auto Name", new PathPlannerAuto("Auto Nickname", bool inversion));

    SmartDashboard.putData("Autonomous Command", autoChooser);
  }

  private void bind(Trigger trigger) {
    boundTriggers.add(trigger);
  }

  public void enableAdjustmentBindings() {
    // all direct robot controls are bound in the state machine already
    bind(driver.start().onTrue(Command.requiring(drive).executing(co -> {
      drive.setPose(new Pose2d(drive.getPose().getTranslation(), new Rotation2d()));
    }).named("RESET HEADING")));

    Trigger operatorLeftYUp = new Trigger(() -> operator.getLeftY() < -0.9);
    bind(operatorLeftYUp.whileTrue(telescope.adjustAngleDeg(10 / Constants.LOOP_FREQ_HZ)));
    Trigger operatorLeftYDown = new Trigger(() -> operator.getLeftY() > 0.9);
    bind(operatorLeftYDown.whileTrue(telescope.adjustAngleDeg(-10 / Constants.LOOP_FREQ_HZ)));
    Trigger operatorRightYUp = new Trigger(() -> operator.getRightY() < -0.9);
    bind(operatorRightYUp.whileTrue(telescope.adjustExtensionIn(1.5 / Constants.LOOP_FREQ_HZ)));
    Trigger operatorRightYDown = new Trigger(() -> operator.getRightY() > 0.9);
    bind(operatorRightYDown.whileTrue(telescope.adjustExtensionIn(-1.5 / Constants.LOOP_FREQ_HZ)));

    bind(operator.povUp().whileTrue(endEffector.adjustAngleDeg(10 / Constants.LOOP_FREQ_HZ)));
    bind(operator.povDown().whileTrue(endEffector.adjustAngleDeg(-10 / Constants.LOOP_FREQ_HZ)));
    bind(operator.povRight().whileTrue(endEffector.adjustVoltage(0.5 / Constants.LOOP_FREQ_HZ)));
    bind(operator.povLeft().whileTrue(endEffector.adjustVoltage(-0.5 / Constants.LOOP_FREQ_HZ)));

    bind(operator.rightBumper().whileTrue(launcher.adjustRPS(1 / Constants.LOOP_FREQ_HZ)));
    bind(operator.leftBumper().whileTrue(launcher.adjustRPS(-1 / Constants.LOOP_FREQ_HZ)));
  }

  public void enableTuningBindings() {
    bind(operator.start().onTrue(SMManager.functional()));
    bind(operator.back().onTrue(SMManager.tuning()));
    bind(driver.back().onTrue(SMManager.teleop()));
  }

  public void enableColorChangeBindings() {
    bind(operator.a().onTrue(endEffector.setCrystalColor(CrystalColor.ORANGE)));
    bind(operator.b().onTrue(endEffector.setCrystalColor(CrystalColor.GREEN)));
    bind(operator.x().onTrue(endEffector.setCrystalColor(CrystalColor.YELLOW)));
    bind(operator.y().onTrue(endEffector.setCrystalColor(CrystalColor.PURPLE)));
  }

  public void unbindAll() {
    for (Trigger trigger : boundTriggers) {
      trigger.unbind();
    }
    boundTriggers = new ArrayList<Trigger>();
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command teleopSM() {
    return SMManager.teleop();
  }

  public Command functionalSM() {
    return SMManager.functional();
  }

  public void periodic() {
    drive.periodic();
    telescope.logIO();
    launcher.logIO();
    endEffector.logIO();
    SMManager.logData();
    if (Constants.currentMode!=Mode.REAL) {
      visualizer2d.updateVis(
        telescope.getPivotAngleDeg(),
        telescope.getArmExtensionInches(),
        endEffector.getWristAngleDeg(),
        launcher.getMeanRPS(),
        endEffector.getRollersRPS()
      );
    }
    PhoenixUtil.refreshAll();
  }

}
