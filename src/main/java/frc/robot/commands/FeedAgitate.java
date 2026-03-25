// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.hopper.Hopper;

/** Feeds game pieces while periodically reversing the hopper to agitate. */
public class FeedAgitate extends Command {
  private final Hopper m_hopper;
  private final Column m_column;
  private final Timer m_timer = new Timer();

  public FeedAgitate(Hopper hopper, Column column) {
    m_hopper = hopper;
    m_column = column;
    addRequirements(hopper, column);
  }

  @Override
  public void initialize() {
    m_timer.restart();
  }

  @Override
  public void execute() {
    // Reverse hopper for 1 second out of every 5-second cycle
    double cyclePosition = m_timer.get() % 5.0;
    if (cyclePosition >= 4.0) {
      m_hopper.setVoltage(-HopperConstants.LAUNCHING_VOLTAGE);
    } else {
      m_hopper.setVoltage(HopperConstants.LAUNCHING_VOLTAGE);
    }
    m_column.setVoltage(ColumnConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.setVoltage(0.0);
    m_column.setVoltage(0);
  }
}
