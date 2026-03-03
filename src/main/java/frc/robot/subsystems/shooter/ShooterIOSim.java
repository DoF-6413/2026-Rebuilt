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
  private DCMotorSim shooterSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, ShooterConstants.GEAR_RATIO),
          DCMotor.getCIM(1));

  private double shooterAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    shooterSim.setInputVoltage(shooterAppliedVolts);
    shooterSim.update(RobotStateConstants.PERIODIC_LOOP_SEC);

    inputs.middleShooterRPM = shooterSim.getAngularVelocityRPM();
    inputs.middleShooterAppliedVolts = shooterAppliedVolts;
    inputs.middleShooterCurrentAmps = shooterSim.getCurrentDrawAmps();
  }

  // @Override
  // public void setVoltage(double volts) {
  //   shooterAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  // }

  @Override
  public void setVelocity(double velocity) {
    shooterSim.setAngularVelocity(velocity);
  }
}
