// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ShooterConstants.SETPOINT_1_RPM;
import static frc.robot.Constants.ShooterConstants.SETPOINT_2_RPM;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.shooter.Shooter;

public class Launch extends Command {
  private final Shooter m_shooter;
  private final CommandXboxController m_controller;
  private final double speed;

  public Launch(Shooter shooter, CommandXboxController controller, String position) {
    m_shooter = shooter;
    m_controller = controller;
    if (position.equals("trench")) {
      speed = SETPOINT_1_RPM;
    } else if (position.equals("hub")) {
      speed = SETPOINT_2_RPM;
    } else {
      speed = 0.0;
    }

    addRequirements(shooter);
  }

  @Override
  public void initialize() {
    m_shooter.setVelocity(speed);
  }

  @Override
  public void execute() {
    m_shooter.setVelocity(speed);
    if (m_shooter.getVelocity() > speed) {
      m_controller.getHID().setRumble(RumbleType.kBothRumble, 1.0);
    } else {
      m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_shooter.setVelocity(0.0);
    m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0);
  }
}
