// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotStateConstants;

public class IntakeIOSim implements IntakeIO {
  private final DCMotorSim m_intakeSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, IntakeConstants.GEAR_RATIO),
          DCMotor.getCIM(1));

  private double m_intakeAppliedVolts = 0.0;

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    m_intakeSim.setInputVoltage(m_intakeAppliedVolts);
    m_intakeSim.update(RobotStateConstants.PERIODIC_LOOP_SEC);

    inputs.intakeRPM = m_intakeSim.getAngularVelocityRPM();
    inputs.intakeAppliedVolts = m_intakeAppliedVolts;
    inputs.intakeCurrentAmps = m_intakeSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    m_intakeAppliedVolts =
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE);
  }
}
