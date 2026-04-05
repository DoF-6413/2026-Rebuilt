// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.generated.TunerConstants;
import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 m_pigeon =
      new Pigeon2(TunerConstants.DrivetrainConstants.Pigeon2Id, TunerConstants.kCANBus);
  private final StatusSignal<Angle> m_yaw = m_pigeon.getYaw();
  private final Queue<Double> m_yawPositionQueue;
  private final Queue<Double> m_yawTimestampQueue;
  private final StatusSignal<AngularVelocity> m_yawVelocity = m_pigeon.getAngularVelocityZWorld();

  public GyroIOPigeon2() {
    if (TunerConstants.DrivetrainConstants.Pigeon2Configs != null) {
      m_pigeon.getConfigurator().apply(TunerConstants.DrivetrainConstants.Pigeon2Configs);
    } else {
      m_pigeon.getConfigurator().apply(new Pigeon2Configuration());
    }

    m_pigeon.getConfigurator().setYaw(180);
    m_yaw.setUpdateFrequency(Drive.ODOMETRY_FREQUENCY);
    m_yawVelocity.setUpdateFrequency(50.0);
    m_pigeon.optimizeBusUtilization();
    m_yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    m_yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(m_yaw.clone());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(m_yaw, m_yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(m_yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(m_yawVelocity.getValueAsDouble());

    inputs.odometryYawTimestamps =
        m_yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        m_yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);
    m_yawTimestampQueue.clear();
    m_yawPositionQueue.clear();
  }
}
