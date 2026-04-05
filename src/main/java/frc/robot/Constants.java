// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.List;
import java.util.Optional;
import java.util.Set;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {

  public static class RobotStateConstants {
    public static final Mode simMode = Mode.SIM;
    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,

      /** Running a physics simulator. */
      SIM,

      /** Replaying from a log file. */
      REPLAY
    }

    /**
     * @return Alliance from FMS
     */
    public static Optional<Alliance> getAlliance() {
      return DriverStation.getAlliance();
    }

    /* Motor Configs */
    /* Refreshes TalonFX signals 50 times a second (every 0.02 seconds) */
    public static final double UPDATE_FREQUENCY_HZ = 50;
    /* Times out PHX tuner config after 0.25 sec */
    public static final double PHX_CONFIG_TIMEOUT_SEC = 0.25;
    /* Times out CAN bus after 30 sec */
    public static final int CAN_CONFIG_TIMEOUT_SEC = 30;

    public static final double PERIODIC_LOOP_SEC = 0.02;
    public static final double MAX_VOLTAGE = 12;

    /** Weight of robot with bumpers and battery */
    public static final double ROBOT_WEIGHT_KG = Units.lbsToKilograms(115.0); // TODO: Update
  }

  public static class OperatorConstants {
    public static int DRIVE_CONTROLLER = 0;
    public static int AUX_CONTROLLER = 1;
  }

  public static class FieldConstants {
    // Hub center positions for auto-aim (field coordinates, meters)
    public static final Translation2d BLUE_HUB_POSITION = new Translation2d(5.02, 4.11);
    public static final Translation2d RED_HUB_POSITION = new Translation2d(11.702, 4.11);
  }

  public static class PathFinderConstants {
    static PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);
    static List<Waypoint> blueHubWaypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(2.00, 4.000, new Rotation2d()), new Pose2d(3.536, 4.000, new Rotation2d()));
    static PathPlannerPath bluePath =
        new PathPlannerPath(
            blueHubWaypoints, constraints, null, new GoalEndState(0.0, new Rotation2d()));
    static List<Waypoint> redHubWaypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(14.500, 4.000, new Rotation2d()),
            new Pose2d(13.000, 4.000, new Rotation2d()));
    static PathPlannerPath redPath =
        new PathPlannerPath(
            redHubWaypoints, constraints, null, new GoalEndState(0.0, new Rotation2d()));
  }

  public static class ColumnConstants {
    public static final int CAN_ID = 18;
    public static final double GEAR_RATIO = 1.0;
    public static final int CURRENT_LIMIT = 40;

    public static final boolean IS_INVERTED = false; // false = CCW
    public static final boolean ENABLE_CURRENT_LIMIT = true;

    public static final double INTAKING_VOLTAGE = -12; // -12.0;
    public static final double LAUNCHING_VOLTAGE = -12; // -12.0;
  }

  public static class HopperConstants {
    public static final int CAN_ID = 19;
    /* CAN ID for Kraken */

    // Geometry for calculations
    /* Gear reduction of 1:1 */
    public static final double GEAR_RATIO = 1.0 / 1.0;

    /* Sets inversion of motor to false, making CCW = positive direction */
    public static final boolean IS_INVERTED = false;
    public static final boolean IS_BRAKE_MODE_ENABLED = false;

    /* Current limiting */
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    public static final int CURRENT_LIMIT = 20;

    public static final double LAUNCHING_VOLTAGE = -12.0;
    public static final double OUTTAKING_VOLTAGE = 3.0;

    /* PID & FF Constants */
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double TOLERANCE_RAD = 0.0;

    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public static double MAX_VELOCITY_DEG_PER_S = 0.0;
    public static double MAX_ACCELERATION_DEG_PER_S2 = 0.0;

    public static double MOI_KG_M2 = 0.0;
  }

  public static class IntakeConstants {
    /* CAN ID for Kraken */
    public static final int CAN_ID = 21;

    // Geometry for calculations
    /* Gear reduction of 1:1 */
    public static final double GEAR_RATIO = 1.0;

    /* Sets inversion of motor to false, making CCW = positive direction */
    public static final boolean IS_INVERTED = false;
    public static final boolean IS_BRAKE_MODE_ENABLED = false;

    /* Current limiting */
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    public static final int CURRENT_LIMIT = 50;

    public static final double INTAKING_VOLTAGE = 12.0;

    /* PID & FF Constants */
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double TOLERANCE_RPM = 0.0;

    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public static double MOI_KG_M2 = 0.0;
  }

  public static class PivotConstants {
    /* CAN ID for Kraken */
    public static final int CAN_ID = 20;

    // Geometry for calculations
    /* Gear reduction of 50:1 */
    public static final double GEAR_RATIO = 50.0 / 1.0;
    /* Length of Pivot in meters */
    public static final double LENGTH_M = Units.inchesToMeters(0.0);
    /* Weight of Pivot in kilograms */
    public static final double WEIGHT_KG = Units.lbsToKilograms(0.0);

    /* Sets inversion of motor to false, making CCW = positive direction */
    public static final boolean IS_INVERTED = false;
    public static final boolean IS_BRAKE_MODE_ENABLED = false;

    /* Current limiting */
    public static final boolean ENABLE_CURRENT_LIMIT = true;
    public static final int CURRENT_LIMIT = 5; // 10 was too much

    /* Angle positions */
    /* These are measured in rotations because that's what Phoenix Tuner X gives them in, and it's also what all the methods ask for */

    /* Angle of the pivot at the start of the match before it's deployed */
    public static final double HOMED_ANGLE_ROT = 0.0;

    /* Angle of the pivot after it's been deployed - used for intaking */
    public static final double DEPLOYED_ANGLE_ROT = 15.5;

    /* About half of the deploued angle; the agitate command should move the pivot from the deployed position to this position */
    public static final double AGITATING_ANGLE_ROT = 7.5;

    public static final double AGITATING_VOLTAGE = 12.0;

    /* PID & FF Constants */
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double TOLERANCE_DEG = 0.0;

    public static double kS = 0.0;
    public static double kV = 12 / 100;
    public static double kA = 0.0;

    public static double MAX_VELOCITY_DEG_PER_S = 0.0;
    public static double MAX_ACCELERATION_DEG_PER_S2 = 0.0;

    public static double MOI_KG_M2 = 0.1;
  }

  public static final class ShooterConstants {
    public static final int MIDDLE_CAN_ID = 15;
    public static final int RIGHT_CAN_ID = 16;
    public static final int LEFT_CAN_ID = 17;
    public static final double GEAR_RATIO = 1.0;
    public static final int CURRENT_LIMIT = 40;
    public static final int STATOR_CURRENT_LIMIT = 120;

    // PID and FF as of 4.5.26
    public static double kP = 0.505;
    public static double kI = 2;
    public static double kD = 0;
    public static double kV = 0.125;

    // Shooter speed setpoints
    public static final double SETPOINT_1_RPM = 3200; // Meant for shooting from hub
    public static final double SETPOINT_2_RPM = 3850; // Meant for shooting from trench
    public static final double SETPOINT_3_RPM = 3300; // Meant for shooting from sides of the tower
    public static double TOLERANCE_RPM = 100; // TODO: verify
  }

  public static final class HoodConstants {
    public static final int leftServoPort = 1;
    public static final int rightServoPort = 2;

    public static final double K_MIN_POSITION = 0.01;
    public static final double K_MAX_POSITION = 0.77;
    public static final double K_TOLERANCE = 0.01;

    // HUB
    public static final double SETPOINT_1 = 0.2;
    // TRENCH
    public static final double SETPOINT_2 = 0.7;
    // TOWER
    public static final double SETPOINT_3 = 0.6;
  }

  public static class VisionConstants {
    // AprilTag layout
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

    // Camera names, must match names configured on coprocessor
    public static String camera1Name = "Camera_1";
    public static String camera2Name = "Camera_2";

    // AprilTag IDs to track for pose estimation (reef and processor tags relevant to gameplay)
    public static final Set<Integer> TRACKED_TAG_IDS =
        Set.of(2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27);

    // Robot to camera transforms
    // Camera translation for left camera
    public static Transform3d robotToCamera1 =
        new Transform3d(
            Units.inchesToMeters(3.75),
            Units.inchesToMeters(12.5),
            Units.inchesToMeters(12.5),
            new Rotation3d(
                Units.degreesToRadians(20.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(90.0)));
    // Camera translation for right camera
    public static Transform3d robotToCamera2 =
        new Transform3d(
            Units.inchesToMeters(3.75),
            Units.inchesToMeters(-11.5),
            Units.inchesToMeters(12.5),
            new Rotation3d(
                Units.degreesToRadians(20.0),
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(-90.0)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.3;
    public static double maxZError = 0.75;

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double linearStdDevBaseline = 0.02; // Meters
    public static double angularStdDevBaseline = 0.06; // Radians

    // Standard deviation multipliers for each camera
    // (Adjust to trust some cameras more than others)
    public static double[] cameraStdDevFactors =
        new double[] {
          1.0, // Camera 0
          1.0 // Camera 1
        };

    // Multipliers to apply for MegaTag 2 observations
    public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
    public static double angularStdDevMegatag2Factor =
        Double.POSITIVE_INFINITY; // No rotation data available
  }
}
