// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package first.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import first.robot.generated.TunerConstants;
import first.robot.subsystems.drive.DriveConstants.GyroInputs;

import java.util.LinkedList;
import java.util.Queue;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> yaw = pigeon.getYaw();
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;
  private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    pigeon.getConfigurator().setYaw(0.0);
    yaw.setUpdateFrequency(DriveConstants.ODOMETRY_FREQUENCY);
    yawVelocity.setUpdateFrequency(50.0);
    pigeon.optimizeBusUtilization();
    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone());
  }

  @Override
  public GyroInputs updateInputs() {
    Queue<Double> timestamps = new LinkedList<Double>(yawTimestampQueue); yawTimestampQueue.clear();
    Queue<Double> positions = new LinkedList<Double>(yawPositionQueue); yawPositionQueue.clear();
    return new GyroInputs(
    BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK),
    Rotation2d.fromDegrees(yaw.getValueAsDouble()),
    Units.degreesToRadians(yawVelocity.getValueAsDouble()),
    timestamps.stream().mapToDouble((Double value) -> value).toArray(),
    positions.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new)
    );
  }
}
