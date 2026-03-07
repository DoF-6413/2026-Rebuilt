// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // Update IOs
  private final ShooterIO m_io;
  private final ShooterIOInputsAutoLogged m_inputs = new ShooterIOInputsAutoLogged();

  // Controllers
  // private final PIDController m_PIDController;
  // private final SimpleMotorFeedforward m_FFController;
  private double m_setpoint = ShooterConstants.SETPOINT_2_RPM;
  // private boolean m_enablePID = true;
  // private boolean m_enableTesting = true;

  public Shooter(ShooterIO io) {
    System.out.println("[INIT] Shooter");
    m_io = io;
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.processInputs("Shooter", m_inputs);

    SmartDashboard.putNumber("Shooter/RPM???", m_setpoint);
  }

  public void setVelocity(double velocityRPM) {
    m_io.setVelocity(velocityRPM);
  }

  public void enableBrakeMode(boolean enable) {
    m_io.enableBrakeMode(enable);
  }

  public double getVelocity() {
    return m_inputs.middleShooterRPS;
  }
}
