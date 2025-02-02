package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.Set;

public class AllianceUtil {
  public static final Set<Integer> RED_REEF_IDS = Set.of(6, 7, 8, 9, 10, 11);
  public static final Set<Integer> BLUE_REEF_IDS = Set.of(17, 18, 19, 20, 21, 22);

  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    return alliance.isPresent() && alliance.get() == Alliance.Red;
  }

  public static Rotation2d getZeroRotation() {
    if (isRedAlliance()) {
      return Rotation2d.fromDegrees(180.0);
    } else {
      return Rotation2d.fromDegrees(0.0);
    }
  }

  public static Set<Integer> getReefIds() {
    return isRedAlliance() ? RED_REEF_IDS : BLUE_REEF_IDS;
  }

  public static Pose2d getReefPose() {
    return isRedAlliance() ? FieldConstants.reefRedAlliance : FieldConstants.reefBlueAlliance;
  }

  public static final Map<Integer, Double> aprilTagAngles = new HashMap<>();

  static {
    aprilTagAngles.put(6, 120.0);
    aprilTagAngles.put(7, 180.0);
    aprilTagAngles.put(8, -120.0);
    aprilTagAngles.put(9, -60.0);
    aprilTagAngles.put(10, 0.0);
    aprilTagAngles.put(11, 60.0);
    aprilTagAngles.put(17, 60.0);
    aprilTagAngles.put(18, 0.0);
    aprilTagAngles.put(19, -60.0);
    aprilTagAngles.put(20, -120.0);
    aprilTagAngles.put(21, 180.0);
    aprilTagAngles.put(22, 120.0);
  }
}
