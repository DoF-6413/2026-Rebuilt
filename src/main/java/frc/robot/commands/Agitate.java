// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Agitate extends Command {
  public Intake m_intake;
  public Pivot m_pivot;
  private Timer m_timer = new Timer();

  /** Creates a new IntakeRetract. */
  public Agitate(Intake intake, Pivot pivot) {
    m_intake = intake;
    m_pivot = pivot;
    addRequirements(intake, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_pivot.getAngle() > (Units.radiansToRotations(PivotConstants.MIN_ANGLE_RAD + 1))) {
      m_pivot.setPosition(Units.radiansToRotations(PivotConstants.MIN_ANGLE_RAD));
    } else if (m_pivot.getAngle()
        < (Units.radiansToRotations(PivotConstants.AGITATING_ANGLE_RAD - 1))) {
      m_pivot.setPosition(Units.radiansToRotations(PivotConstants.AGITATING_ANGLE_RAD));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
}
