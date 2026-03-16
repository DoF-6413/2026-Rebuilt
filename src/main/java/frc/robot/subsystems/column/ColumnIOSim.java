// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.column;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.RobotStateConstants;

public class ColumnIOSim implements ColumnIO {
  private final DCMotorSim m_columnSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, ColumnConstants.GEAR_RATIO),
          DCMotor.getCIM(1));

  private double m_columnAppliedVolts = 0.0;

  @Override
  public void updateInputs(ColumnIOInputs inputs) {
    m_columnSim.setInputVoltage(m_columnAppliedVolts);
    m_columnSim.update(RobotStateConstants.PERIODIC_LOOP_SEC);

    inputs.columnRPM = m_columnSim.getAngularVelocityRPM();
    inputs.columnAppliedVolts = m_columnAppliedVolts;
    inputs.columnCurrentAmps = m_columnSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    m_columnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}
