// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotStateConstants;

public class HopperIOSim implements HopperIO {
  private DCMotorSim hopperSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getKrakenX60(1), 0.004, HopperConstants.GEAR_RATIO),
          DCMotor.getKrakenX60(1));

  private double hopperAppliedVolts = 0.0;

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    hopperSim.setInputVoltage(hopperAppliedVolts);
    hopperSim.update(RobotStateConstants.PERIODIC_LOOP_SEC);

    inputs.hopperRPM = hopperSim.getAngularVelocityRPM();
    inputs.hopperAppliedVolts = hopperAppliedVolts;
    inputs.hopperCurrentAmps = hopperSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    hopperAppliedVolts =
        MathUtil.clamp(volts, -RobotStateConstants.MAX_VOLTAGE, RobotStateConstants.MAX_VOLTAGE);
  }
}
