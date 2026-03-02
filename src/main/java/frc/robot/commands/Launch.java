// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.SPINUP_SEC;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class Launch extends Command {
  public Shooter m_shooter;
  public Column m_column;
  public Hopper m_hopper;

  public Launch(Shooter shooter, Column column, Hopper hopper) {
    m_shooter = shooter;
    m_column = column;
    m_hopper = hopper;

    addRequirements(shooter, column, hopper);
  }

  @Override
  public void initialize() {
    m_column.setVoltage(ColumnConstants.INTAKING_VOLTAGE);
    m_shooter.setVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void execute() {
    new WaitCommand(SPINUP_SEC);
    m_column.setVoltage(ColumnConstants.LAUNCHING_VOLTAGE);
    m_shooter.setVoltage(ShooterConstants.LAUNCHING_VOLTAGE);
  }

  @Override
  public void end(boolean interrupted) {
    m_column.setVoltage(0.0);
    m_shooter.setVoltage(0.0);
  }
}
