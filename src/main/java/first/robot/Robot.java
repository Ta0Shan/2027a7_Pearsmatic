// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Nanoseconds;
import static org.wpilib.units.Units.Seconds;

import java.util.ArrayList;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.opmode.OpMode;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

import first.robot.Constants.CrystalColor;
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
import first.robot.util.PhoenixUtil;

public class Robot extends OpModeRobot {
  private final long startTimestamp = RobotController.getMonotonicTime();
  
  public final CommandXboxController driver = new CommandXboxController(0);
  public final CommandXboxController operator = new CommandXboxController(1);

  public final CommandXboxController keyboard = new CommandXboxController(2);

  public final Drive drive;
  public final Telescope telescope;
  public final EE endEffector;
  public final Launcher launcher;

  public final StateMachineManager SMManager;
  private final MechVisualizer visualizer;

  private final ArrayList<String> issues = new ArrayList<String>();

  public Robot() {

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

    
    SMManager = new StateMachineManager(
        telescope,
        launcher,
        endEffector,
        drive,

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


    Scheduler.getDefault().addPeriodic(() -> {
      PhoenixUtil.refreshAll();
      telescope.logIO();
      launcher.logIO();
      endEffector.logIO();
      drive.periodic();
      SMManager.logAdditionalData();
    });
    visualizer = new MechVisualizer();


    addAuto(Command.noRequirements(co -> {}).named("auto1"));
    addAuto(Command.noRequirements(co -> {}).named("auto2"));
    addAuto(Command.noRequirements(co -> {}).named("auto3"));

    publishOpModes();
  }

  @Override
  public void robotPeriodic() {
    Command[] runningCommands = Scheduler.getDefault().getRunningCommands().toArray(new Command[] {});
    String[] names = new String[runningCommands.length];
    for (int i = 0; i < names.length; i++) {
      names[i] = runningCommands[i].name();
    }
    Telemetry.log("Commands/Running", names);

    Scheduler.getDefault().addEventListener(event -> {
      String message;
      // Alert alert;
      switch(event) {
        case SchedulerEvent.Interrupted(Command cmd, Command other, long time_ns):
          // alert = new Alert(Seconds.convertFrom(time_ns, Nanoseconds) + "", other.name() + " interrupted by " + cmd.name(), Level.LOW);
          message = Seconds.convertFrom(time_ns-startTimestamp, Nanoseconds) + ": " + cmd.name() + " interrupted by " + other.name();
          if(!issues.contains(message)) issues.addFirst(message);
          break;
        case SchedulerEvent.CompletedWithError(Command cmd, Error e, long time_ns):
          // alert = new Alert(Seconds.convertFrom(time_ns, Nanoseconds) + "", cmd.name() + " completed with error " + e.getLocalizedMessage(), Level.LOW);
          message = Seconds.convertFrom(time_ns-startTimestamp, Nanoseconds) + ": " + cmd.name() + "completed with error " + e.getMessage();
          if(!issues.contains(message)) issues.addFirst(message);
        break;
        // case SchedulerEvent.Canceled(Command cmd, long time_ns):
        //   // alert = new Alert(Seconds.convertFrom(time_ns, Nanoseconds) + "", cmd.name() + " canceled ", Level.LOW);
        //   message = Seconds.convertFrom(time_ns, Nanoseconds)/100 + ": " + cmd.name() + " cancelled";
        //   if(!issues.contains(message)) issues.addFirst(message);
        //   break;
        default:
          // alert = null;
          break;
      }
      // if (alert != null) alert.set(true);
    });
    Telemetry.log("Commands/Issues", issues.toArray(new String[] {}));

    Scheduler.getDefault().run();
  }

  @Override
  public void simulationInit() {
    Scheduler.getDefault().addPeriodic(
      () -> visualizer.updateVis(
        telescope.getPivotAngleDeg(),
        telescope.getArmExtensionInches(),
        endEffector.getWristAngleDeg(),
        launcher.getMeanRPS(),
        endEffector.getRollersRPS()
      )
    );

    keyboard.a().and(keyboard.b().negate()).onTrue(endEffector.setCrystalColor(CrystalColor.ORANGE));
    keyboard.b().and(keyboard.a().negate()).onTrue(endEffector.setCrystalColor(CrystalColor.GREEN));
    keyboard.x().onTrue(endEffector.setCrystalColor(CrystalColor.YELLOW));
    keyboard.y().onTrue(endEffector.setCrystalColor(CrystalColor.PURPLE));

    keyboard.a().and(keyboard.b()).onTrue(endEffector.setCrystalColor(CrystalColor.NONE));
  }

  @Override
  public void simulationPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void nonePeriodic() {}

  private void addAuto(Command autoCommand) {
    addOpMode(RobotMode.AUTONOMOUS, autoCommand.name(), () -> OpModeContainer.generateAuto(autoCommand));
  }

}