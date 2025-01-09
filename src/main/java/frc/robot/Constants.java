package frc.robot;

import edu.wpi.first.math.util.Units;

public class Constants {
  public class SwerveConstants {
    public static final double maxTranslationalSpeed = 5;
    public static final double maxRotationalSpeed = 2;

    public static final double maxTranslationalAcceleration = Units.feetToMeters(18);
    public static final double maxRotationalAcceleration = Units.feetToMeters(270);
  }
}
