package frc.robot.util;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.HoodConstants;
import frc.robot.Constants.ShooterConstants;

public class ShotMapUtil {

    // Lookup table mapping shooting distance in meters to Shot parameters (RPM + hood position).
  // We use interpolation between known distances so we can smoothly compute
  // appropriate shooter settings for any intermediate distance.
  public static final InterpolatingTreeMap<Distance, Shot> distanceToShotMap =
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
        Inches.of(47.96), new Shot(ShooterConstants.HUB_SPEED_RPM, HoodConstants.HUB_SETPOINT));
    distanceToShotMap.put(
        Inches.of(122.12),
        new Shot(ShooterConstants.TOWER_SPEED_RPM, HoodConstants.TOWER_SETPOINT));
    distanceToShotMap.put(
        Inches.of(134.13),
        new Shot(ShooterConstants.TRENCH_SPEED_RPM, HoodConstants.TRENCH_SETPOINT));
    distanceToShotMap.put(
        Inches.of(192.0),
        new Shot(ShooterConstants.CORNER_SPEED_RPM, HoodConstants.CORNER_SETPOINT));
  }

  public static Distance getDistanceToHub(Supplier<Pose2d> m_poseSupplier, Translation2d target) {
    Translation2d robotPosition = m_poseSupplier.get().getTranslation();
    return Meters.of(robotPosition.getDistance(target));
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
