package frc.robot;

import static edu.wpi.first.units.Units.FeetPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public class Constants {
  public static class SwerveConstants {
    public static final LinearVelocity maxTranslationalSpeed = FeetPerSecond.of(15);
    public static final AngularVelocity maxRotationalSpeed = RotationsPerSecond.of(1);

    public static final Time translationZeroToFull = Seconds.of(.5);
    public static final Time rotationZeroToFull = Seconds.of(0.25);

    public static final LinearAcceleration maxTransationalAcceleration =
        maxTranslationalSpeed.div(translationZeroToFull);
    public static final AngularAcceleration maxAngularAcceleration =
        maxRotationalSpeed.div(rotationZeroToFull);
  }

  public static class AutoConstants {
    public static final PIDConstants translationPID = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants rotationPID = new PIDConstants(1.0, 0.0, 0.0);
  }

  public static class FieldConstants {
    public static AprilTagFieldLayout aprilTagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  }

  public static class IntakeConstants {
    public static final int mainIntakeMotorID = 20;
    public static final int armIntakeMotorID = 21;
    public static final int indexerMotorID = 22;

    public static final int intakeLaserCanID = 14;
    public static final int outakeLaserCanID = 15;

    public static final double indexerMotorSpeeds = .5;
  }
}
