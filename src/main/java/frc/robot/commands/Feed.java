// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.hopper.Hopper;

public class Feed extends Command {
  public Hopper m_hopper;
  public Column m_column;

  public Feed(Hopper hopper, Column column) {
    m_hopper = hopper;
    m_column = column;
    addRequirements(hopper, column);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_hopper.setVoltage(HopperConstants.LAUNCHING_VOLTAGE);
    m_column.setVoltage(ColumnConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.setVoltage(0.0);
    m_column.setVoltage(0);
  }
}
