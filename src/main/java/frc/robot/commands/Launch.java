// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.SETPOINT_RPM;
import static frc.robot.Constants.ShooterConstants.SPINUP_SEC;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class Launch extends Command {
  private final Shooter m_shooter;
  private final Column m_column;
  private final Hopper m_hopper;
  private final Timer m_timer = new Timer();

  public Launch(Shooter shooter, Column column, Hopper hopper) {
    m_shooter = shooter;
    m_column = column;
    m_hopper = hopper;

    addRequirements(shooter, column, hopper);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_shooter.setVelocity(SETPOINT_RPM);
  }

  @Override
  public void execute() {
    if (m_timer.hasElapsed(SPINUP_SEC)) {
      m_column.setVoltage(ColumnConstants.LAUNCHING_VOLTAGE);
      m_hopper.setVoltage(HopperConstants.LAUNCHING_VOLTAGE);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_hopper.setVoltage(0.0);
    m_column.setVoltage(0.0);
    m_shooter.setVelocity(0.0);
  }
}
