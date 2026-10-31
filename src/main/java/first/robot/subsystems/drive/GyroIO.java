// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.math.geometry.Rotation2d;

import first.robot.subsystems.drive.DriveConstants.GyroInputs;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = Rotation2d.ZERO;
    public double yawVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
  }

  public default GyroInputs updateInputs() {return new GyroInputs();}
}
