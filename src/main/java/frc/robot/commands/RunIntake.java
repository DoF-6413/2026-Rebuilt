// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

public class RunIntake extends Command {
  public Intake m_intake;
  public Pivot m_pivot;

  public RunIntake(Intake intake, Pivot pivot) {
    m_intake = intake;
    m_pivot = pivot;
    addRequirements(intake, pivot);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_pivot.d();
    m_intake.setVoltage(-12.0);
  }

  @Override
  public void end(boolean interrupted) {
    m_intake.setVoltage(0.0);
  }
}
