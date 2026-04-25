// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
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
  private final Timer m_timer = new Timer();
  private final double m_timeDelay = 1.5;

  public Relay(Shooter shooter, Hopper hopper, Column column, Hood hood) {
    m_shooter = shooter;
    m_column = column;
    m_hopper = hopper;
    m_hood = hood;

    addRequirements(shooter, hopper, column, hood);
  }

  @Override
  public void initialize() {
    m_timer.restart();

    m_hopper.setVoltage(-HopperConstants.LAUNCHING_VOLTAGE);
    m_shooter.setVoltage(RobotStateConstants.MAX_VOLTAGE);
    m_hood.setPosition(HoodConstants.K_MAX_POSITION);
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(m_timeDelay)) {
      m_hopper.setVoltage(HopperConstants.LAUNCHING_VOLTAGE);
      m_column.setVoltage(ColumnConstants.LAUNCHING_VOLTAGE);
    }
  }

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

    m_timer.stop();
  }
}
