// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.SETPOINT_1_RPM;
import static frc.robot.Constants.ShooterConstants.SETPOINT_2_RPM;
import static frc.robot.Constants.ShooterConstants.SETPOINT_3_RPM;
import static frc.robot.Constants.ShooterConstants.TOLERANCE_RPM;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class Launch extends Command {
  private final Shooter m_shooter;
  private final Column m_column;
  private final Hopper m_hopper;
  private final frc.robot.subsystems.hood.Hood m_hood;
  private final double speed;
  private final double hoodSetpoint;

  public Launch(Shooter shooter, Hopper hopper, Column column, Hood hood, String position) {
    m_shooter = shooter;
    m_column = column;
    m_hopper = hopper;
    m_hood = hood;
    if (position.equals("trench")) {
      speed = SETPOINT_1_RPM;
      hoodSetpoint = 0.5;
    } else if (position.equals("hub")) {
      speed = SETPOINT_2_RPM;
      hoodSetpoint = 0.0;
    } else if (position.equals("tower")) {
      speed = SETPOINT_3_RPM;
      hoodSetpoint = 0.35;
    } else {
      speed = 0.0;
      hoodSetpoint = 0.0;
    }

    addRequirements(shooter, hopper, column, hood);
  }

  @Override
  public void initialize() {
    m_shooter.setVelocity(speed);
    m_hood.setPosition(hoodSetpoint);
  }

  @Override
  public void execute() {
    if (m_shooter.getVelocity() > (speed - TOLERANCE_RPM)) {
      m_hopper.setVoltage(HopperConstants.LAUNCHING_VOLTAGE);
      m_column.setVoltage(ColumnConstants.LAUNCHING_VOLTAGE);
    }
  }

  // public void isFinished()

  @Override
  public void end(boolean interrupted) {
    m_shooter.setVelocity(0.0);
    m_hopper.setVoltage(0.0);
    m_column.setVoltage(0.0);
  }
}
