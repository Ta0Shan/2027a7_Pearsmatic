// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import first.robot.Constants;
import first.robot.subsystems.drive.DriveConstants.ModuleInputs;

import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.math.system.Models;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

/**
 * Physics sim implementation of module IO. The sim models are configured using a set of module
 * constants from Phoenix. Simulation is always based on voltage control.
 */
public class ModuleIOSim implements ModuleIO {

  private final DCMotorSim driveSim;
  private final DCMotorSim turnSim;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private PIDController driveController = new PIDController(DriveConstants.SIM_DRIVE_KP, 0, DriveConstants.SIM_DRIVE_KD);
  private PIDController turnController = new PIDController(DriveConstants.SIM_TURN_KP, 0, DriveConstants.SIM_TURN_KD);
  private double driveFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    // Create drive and turn sim models
    driveSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                DriveConstants.SIM_DRIVE_GEARBOX, constants.DriveInertia, constants.DriveMotorGearRatio),
            DriveConstants.SIM_DRIVE_GEARBOX);
    turnSim =
        new DCMotorSim(
            Models.singleJointedArmFromPhysicalConstants(
                DriveConstants.SIM_TURN_GEARBOX, constants.SteerInertia, constants.SteerMotorGearRatio),
            DriveConstants.SIM_TURN_GEARBOX);

    // Enable wrapping for turn PID
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public ModuleInputs updateInputs() {
    // Run closed-loop control
    if (driveClosedLoop) {
      driveAppliedVolts =
          driveFFVolts + driveController.calculate(driveSim.getAngularVelocity());
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnController.calculate(turnSim.getAngularPosition());
    } else {
      turnController.reset();
    }

    // Update simulation state
    driveSim.setInputVoltage(Math.clamp(driveAppliedVolts, -12.0, 12.0));
    turnSim.setInputVoltage(Math.clamp(turnAppliedVolts, -12.0, 12.0));
    driveSim.update(Constants.LOOP_PERIOD_SEC);
    turnSim.update(Constants.LOOP_PERIOD_SEC);

    return new ModuleInputs(
      // Update drive inputs
      true,
      driveSim.getAngularPosition(),
      driveSim.getAngularVelocity(),
      driveAppliedVolts,
      Math.abs(driveSim.getCurrentDraw()),

      // Update turn inputs
      true,
      true,
      new Rotation2d(turnSim.getAngularPosition()),
      new Rotation2d(turnSim.getAngularPosition()),
      turnSim.getAngularVelocity(),
      turnAppliedVolts,
      Math.abs(turnSim.getCurrentDraw()),

      // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
      // matter)
      new double[] {Timer.getMonotonicTimestamp()},
      new double[] {driveSim.getAngularPosition()},
      new Rotation2d[] {new Rotation2d(turnSim.getAngularPosition())}
    );
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DriveConstants.SIM_DRIVE_KS * Math.signum(velocityRadPerSec) + DriveConstants.SIM_DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnPosition(Rotation2d rotation) {
    turnClosedLoop = true;
    turnController.setSetpoint(rotation.getRadians());
  }

}