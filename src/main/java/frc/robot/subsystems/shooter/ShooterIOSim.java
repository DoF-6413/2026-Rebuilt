// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim m_shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, ShooterConstants.GEAR_RATIO),
          DCMotor.getCIM(1));

  private double m_shooterAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    m_shooterSim.setInputVoltage(m_shooterAppliedVolts);
    m_shooterSim.update(RobotStateConstants.PERIODIC_LOOP_SEC);

    inputs.middleShooterRPS = m_shooterSim.getAngularVelocityRPM() / 60.0;
    inputs.middleShooterAppliedVolts = m_shooterAppliedVolts;
    inputs.middleShooterCurrentAmps = m_shooterSim.getCurrentDrawAmps();
  }

  @Override
  public void setVelocity(double velocityRPM) {
    m_shooterSim.setAngularVelocity(velocityRPM / 60.0);
  }
}
