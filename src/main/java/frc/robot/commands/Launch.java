// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ColumnConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.column.Column;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.ShotMapUtil;
import frc.robot.util.ShotMapUtil.Shot;

import java.util.function.Supplier;

public class Launch extends Command {
  private final Shooter m_shooter;
  private final Column m_column;
  private final Hopper m_hopper;
  private final Hood m_hood;
  private final Supplier<Pose2d> m_poseSupplier;
  private Shot m_shot;
  private final String m_position;
  private Translation2d m_target = FieldConstants.BLUE_HUB_POSITION;

  public Launch(
      Shooter shooter,
      Hopper hopper,
      Column column,
      Hood hood,
      Supplier<Pose2d> robotPose,
      String position) {
    m_shooter = shooter;
    m_column = column;
    m_hopper = hopper;
    m_hood = hood;
    m_poseSupplier = robotPose;
    m_position = position;

    addRequirements(shooter, hopper, column, hood);
  }

  @Override
  public void initialize() {
    // Resolve alliance here (not at construction time) so DS has confirmed alliance
    m_target =
        DriverStation.getAlliance()
            .filter(a -> a == Alliance.Red)
            .map(a -> FieldConstants.RED_HUB_POSITION)
            .orElse(FieldConstants.BLUE_HUB_POSITION);

    m_hopper.setVoltage(-HopperConstants.LAUNCHING_VOLTAGE);

    if (m_position.equals("trench")) {
      m_shot = new Shot(ShooterConstants.TRENCH_SPEED_RPM, HoodConstants.TRENCH_SETPOINT);
    } else if (m_position.equals("hub")) {
      m_shot = new Shot(ShooterConstants.HUB_SPEED_RPM, HoodConstants.HUB_SETPOINT);
    } else if (m_position.equals("tower")) {
      m_shot = new Shot(ShooterConstants.TOWER_SPEED_RPM, HoodConstants.TOWER_SETPOINT);
    } else if (m_position.equals("corner")) {
      m_shot = new Shot(ShooterConstants.CORNER_SPEED_RPM, HoodConstants.CORNER_SETPOINT);
    } else {
      m_shot = ShotMapUtil.distanceToShotMap.get(ShotMapUtil.getDistanceToHub(m_poseSupplier, m_target));
    }
  }

  @Override
  public void execute() {
    m_shooter.setVelocity(m_shot.m_shooterRPM);
    m_hood.setPosition(m_shot.m_hoodPosition);

    if ((m_shooter.getLVelocity() > (m_shot.m_shooterRPM - ShooterConstants.TOLERANCE_RPM))
        && (m_shooter.getMVelocity() > (m_shot.m_shooterRPM - ShooterConstants.TOLERANCE_RPM))
        && (m_shooter.getRVelocity() > (m_shot.m_shooterRPM - ShooterConstants.TOLERANCE_RPM))) {
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
    m_shooter.setVelocity(0.0);
    m_hopper.setVoltage(0.0);
    m_column.setVoltage(0.0);
    m_hood.setPosition(HoodConstants.K_MIN_POSITION);
  }
}
