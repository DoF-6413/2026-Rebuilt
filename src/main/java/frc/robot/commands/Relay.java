// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class Relay extends Command {
  private final Shooter m_shooter;
  private final Column m_column;
  private final Hopper m_hopper;
  private final Hood m_hood;

  public Relay(Shooter shooter, Hopper hopper, Column column, Hood hood) {
    m_shooter = shooter;
    m_column = column;
    m_hopper = hopper;
    m_hood = hood;

    addRequirements(shooter, hopper, column, hood);
  }

  @Override
  public void initialize() {
    m_shooter.setVoltage(RobotStateConstants.MAX_VOLTAGE);
    m_hood.setPosition(HoodConstants.RELAY_SETPOINT);
    m_hopper.setVoltage(HopperConstants.LAUNCHING_VOLTAGE);
    m_column.setVoltage(ColumnConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void execute() {}

  @Override
  public boolean isFinished() {
    return false; // Command never finishes, its just interrupted
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setVoltage(0.0);
    m_hopper.setVoltage(0.0);
    m_column.setVoltage(0.0);
    m_hood.setPosition(HoodConstants.K_MIN_POSITION);
  }
}
