// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.RobotStateConstants;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;

public class RelayArming extends Command {
  private final Shooter m_shooter;
  private final Hood m_hood;
  private final Hopper m_hopper;

  public RelayArming(Shooter shooter, Hood hood, Hopper hopper) {
    m_shooter = shooter;
    m_hood = hood;
    m_hopper = hopper;

    addRequirements(shooter, hood, hopper);
  }

  @Override
  public void initialize() {
    m_hopper.setVoltage(-HopperConstants.LAUNCHING_VOLTAGE / 2.0);
    m_shooter.setVoltage(RobotStateConstants.MAX_VOLTAGE);
    m_hood.setPosition(HoodConstants.K_MAX_POSITION);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false; // Command never finishes, its just interrupted
  }
}
