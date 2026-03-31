// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static frc.robot.Constants.ShooterConstants.SETPOINT_1_RPM;
import static frc.robot.Constants.ShooterConstants.SETPOINT_2_RPM;
import static frc.robot.Constants.ShooterConstants.SETPOINT_3_RPM;
import static frc.robot.Constants.ShooterConstants.TOLERANCE_RPM;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
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
import java.util.function.Supplier;

public class Launch extends Command {
  private final Shooter m_shooter;
  private final Column m_column;
  private final Hopper m_hopper;
  private final frc.robot.subsystems.hood.Hood m_hood;
  private final Supplier<Pose2d> m_poseSupplier;
  private final Shot m_shot;
  private final Timer m_timer = new Timer();

  private static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
      new InterpolatingTreeMap<>(
          (startValue, endValue, q) ->
              InverseInterpolator.forDouble()
                  .inverseInterpolate(startValue.in(Meters), endValue.in(Meters), q.in(Meters)),
          (startValue, endValue, t) ->
              new Shot(
                  Interpolator.forDouble()
                      .interpolate(startValue.m_shooterRPM, endValue.m_shooterRPM, t),
                  Interpolator.forDouble()
                      .interpolate(startValue.m_hoodPosition, endValue.m_hoodPosition, t)));

  static {
    distanceToShotMap.put(
        Inches.of(64.96), new Shot(ShooterConstants.SETPOINT_1_RPM, HoodConstants.SETPOINT_1));
    distanceToShotMap.put(
        Inches.of(114.4), new Shot(ShooterConstants.SETPOINT_2_RPM, HoodConstants.SETPOINT_2));
    distanceToShotMap.put(
        Inches.of(165.5), new Shot(ShooterConstants.SETPOINT_3_RPM, HoodConstants.SETPOINT_3));
  }

  private Translation2d m_target =
      DriverStation.getAlliance()
          .filter(a -> a == Alliance.Red)
          .map(a -> FieldConstants.RED_HUB_POSITION)
          .orElse(FieldConstants.BLUE_HUB_POSITION);

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
    if (position.equals("trench")) {
      m_shot = new Shot(SETPOINT_1_RPM, HoodConstants.SETPOINT_1);
    } else if (position.equals("hub")) {
      m_shot = new Shot(SETPOINT_2_RPM, HoodConstants.SETPOINT_2);
    } else if (position.equals("tower")) {
      m_shot = new Shot(SETPOINT_3_RPM, HoodConstants.SETPOINT_3);
    } else {
      m_shot = distanceToShotMap.get(getDistanceToHub());
    }

    addRequirements(shooter, hopper, column, hood);
  }

  @Override
  public void initialize() {
    m_shooter.setVelocity(m_shot.m_shooterRPM);
    m_hood.setPosition(m_shot.m_hoodPosition);
    m_timer.restart();
  }

  @Override
  public void execute() {
    m_hood.setPosition(m_shot.m_hoodPosition);
    if ((m_shooter.getVelocity() > (m_shot.m_shooterRPM - TOLERANCE_RPM))
        && m_timer.hasElapsed(1.7)) {
      m_hopper.setVoltage(HopperConstants.LAUNCHING_VOLTAGE);
      m_column.setVoltage(ColumnConstants.LAUNCHING_VOLTAGE);
    }
  }

  // public void isFinished()

  @Override
  public void end(boolean interrupted) {
    m_shooter.setVelocity(0.0);
    m_hopper.setVoltage(0.0);
    m_column.setVoltage(0.0);
    m_hood.setPosition(HoodConstants.K_MIN_POSITION);
  }

  public Distance getDistanceToHub() {
    Translation2d robotPosition = m_poseSupplier.get().getTranslation();
    return Meters.of(robotPosition.getDistance(m_target));
  }

  public static class Shot {
    public final double m_shooterRPM;
    public final double m_hoodPosition;

    public Shot(double shooterRPM, double hoodPosition) {
      m_shooterRPM = shooterRPM;
      m_hoodPosition = hoodPosition;
    }
  }
}
