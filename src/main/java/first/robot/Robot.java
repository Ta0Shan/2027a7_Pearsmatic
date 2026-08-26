// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.SchedulerEvent.CompletedWithError;
import org.wpilib.command3.SchedulerEvent.Interrupted;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import first.robot.Constants.FieldConstants.BlueFieldConstants;
import first.robot.Constants.FieldConstants.RedFieldConstants;

import org.wpilib.command3.SchedulerEvent.Canceled;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  private final Scheduler scheduler;

  private final List<String> problemCommands;

  public Robot() {
    robotContainer = new RobotContainer();

    scheduler = Scheduler.getDefault();
    scheduler.addPeriodic(() -> robotContainer.periodic());

    problemCommands = new ArrayList<String>();

    Logger.addDataReceiver(new NT4Publisher());
    Logger.start();

    // Logger.recordOutput("Field/Origin", FieldConstants.ORIGIN);
    // Logger.recordOutput("Field/Center", new Pose2d(FieldConstants.CENTER, Rotation2d.kZero));
    
    // Logger.recordOutput("Field/Blue/Cave Center", new Pose2d(BlueFieldConstants.CAVE_CENTER, Rotation2d.kZero));
    // Logger.recordOutput("Field/Blue/Lower Shaft Faces", BlueFieldConstants.LOWER_SHAFT_FACES);
    // Logger.recordOutput("Field/Blue/Upper Shaft Vertices", BlueFieldConstants.UPPER_SHAFT_VERTICES);
    // Logger.recordOutput("Field/Blue/Classifier", new Translation2d[] {BlueFieldConstants.CLASSIFIER_SOURCE_CORNER, BlueFieldConstants.CLASSIFIER_MINE_CORNER});
    // Logger.recordOutput("Field/Blue/Classifier Center", BlueFieldConstants.CLASSIFIER_CENTER);
    // Logger.recordOutput("Field/Blue/Mine", new Translation2d[] {BlueFieldConstants.MINE_CENTER, BlueFieldConstants.MINE_CENTER_CORNER, BlueFieldConstants.MINE_DS_CORNER});
    // Logger.recordOutput("Field/Blue/Source", new Translation2d[] {BlueFieldConstants.SOURCE_CENTER, BlueFieldConstants.SOURCE_DS_CORNER, BlueFieldConstants.SOURCE_WALL_CORNER});

  }

  @Override
  public void robotPeriodic() {
    scheduler.run();

    Command[] runningCommands = scheduler.getRunningCommands().toArray(new Command[] {});
    String[] names = new String[runningCommands.length];
    for (int i = 0; i < runningCommands.length; i++) {
      names[i] = runningCommands[i].name();
    }
    
    Logger.recordOutput("Commands/Running", names);

    // outputs important events: non-idle interruptions and errored completions
    scheduler.addEventListener(event -> {
      String message;
      switch(event) {
        case CompletedWithError(Command cmd, Error error, long time):
          message = ((double)Math.round(time / 10000.0) / 100.0) + " | Error with " + cmd.name() + ": " + error.toString();
          if (!problemCommands.contains(message)) {
            problemCommands.add(0, message);
          }
          break;
        case Interrupted(Command cmd, Command inter, long time):
          message = ((double)Math.round(time / 10000.0) / 100.0) + " | " + cmd.name() + " interrupted by " + inter.name();
          if(!cmd.name().contains("[IDLE]") && !problemCommands.contains(message)) {
            problemCommands.add(0, message);
          }
          break;
          // cancellations not rly that important but if necessary we can add
          // case Canceled(Command cmd, long time):
          //   message = ((double)Math.round(time / 10000.0) / 100.0) + " | " + cmd.name() + " canceled";
          //   if(!cmd.name().contains("[IDLE]") && !problemCommands.contains(message)) {
          //     problemCommands.add(0, message);
          //   }
          // break;
        default:
          break;
      }
    });

    Logger.recordOutput("Commands/Special Events/List", problemCommands.toArray(String[]::new));
    Logger.recordOutput("Commands/Special Events/Recent ", (problemCommands.size() > 0 ? problemCommands.get(0) : ""));
  }

  @Override
  public void disabledInit() {
    scheduler.cancelAll();
    robotContainer.unbindAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    if (autonomousCommand != null) {
      scheduler.schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      scheduler.cancel(autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    scheduler.schedule(robotContainer.teleopSM());
    robotContainer.teleopBindings();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
    scheduler.cancelAll();
  }

  @Override
  public void utilityInit() {
    robotContainer.utilityBindings();
    // robotContainer.enableAdjustmentBindings();
  }

  @Override
  public void utilityPeriodic() {}

  @Override
  public void utilityExit() {}
}
