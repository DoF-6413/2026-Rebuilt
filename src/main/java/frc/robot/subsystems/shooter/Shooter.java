// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // Update IOs
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    System.out.println("[INIT] Shooter");
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);
  }

  public void setVelocity(double velocityRPM) {
    m_io.setVelocity(velocityRPM);
  }

  public void setVoltage(double volts) {
    m_io.setVoltage(volts);
  }

  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  /**
   * Gets the current left shooter wheel velocity in RPM.
   *
   * @return Left shooter wheel velocity in RPM
   */
  public double getLVelocity() {
    // Convert the velocity from rotations per second (RPS) to rotations per minute (RPM)
    return m_inputs.leftShooterRPS * 60.0;
  }

  /**
   * Gets the current middle shooter wheel velocity in RPM.
   *
   * @return Middle shooter wheel velocity in RPM
   */
  public double getMVelocity() {
    // Convert the velocity from rotations per second (RPS) to rotations per minute (RPM)
    return m_inputs.middleShooterRPS * 60.0;
  }

  /**
   * Gets the current right shooter wheel velocity in RPM.
   *
   * @return Right shooter wheel velocity in RPM
   */
  public double getRVelocity() {
    // Convert the velocity from rotations per second (RPS) to rotations per minute (RPM)
    return m_inputs.rightShooterRPS * 60.0;
  }
}
