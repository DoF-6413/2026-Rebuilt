// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeRetract extends Command {
  public Intake m_intake;
  public Pivot m_pivot;

  /** Creates a new IntakeRetract. */
  public IntakeRetract(Intake intake, Pivot pivot) {
    m_intake = intake;
    m_pivot = pivot;
    addRequirements(intake, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_pivot.setPosition(PivotConstants.HOMED_ANGLE_ROT);
    m_pivot.setVoltage(2.0);
    m_intake.setVoltage(0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setVoltage(0.0);
    m_pivot.setVoltage(0.0);
  }
}
