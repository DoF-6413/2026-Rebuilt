// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Eject extends Command {
  private final Column m_column;
  private final Hopper m_hopper;
  private final Intake m_intake;

  /** Creates a new Eject. */
  public Eject(Column column, Hopper hopper, Intake intake) {
    m_column = column;
    m_hopper = hopper;
    m_intake = intake;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(column, hopper, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_column.setVoltage(-ColumnConstants.INTAKING_VOLTAGE);
    m_hopper.setVoltage(HopperConstants.OUTTAKING_VOLTAGE);
    m_intake.setVoltage(IntakeConstants.INTAKING_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_column.setVoltage(0.0);
    m_hopper.setVoltage(0.0);
    m_intake.setVoltage(0.0);
  }
}
