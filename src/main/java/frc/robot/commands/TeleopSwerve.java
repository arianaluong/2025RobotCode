// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.function.DoubleSupplier;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TeleopSwerve extends Command {
  /** Creates a new TeleopSwerve. */
  private final CommandSwerveDrivetrain swerve;

  private SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);
  // .withSteerRequestType(DriveRequestType.OpenLoopVoltage);

  DoubleSupplier forwardSupplier;
  DoubleSupplier strafeSupplier;
  DoubleSupplier rotationSupplier;

  SlewRateLimiter forwardRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);
  SlewRateLimiter strafeRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxTranslationalAcceleration);
  SlewRateLimiter rotationRateLimiter =
      new SlewRateLimiter(SwerveConstants.maxRotationalAcceleration);

  public TeleopSwerve(
      DoubleSupplier forwardSupplier,
      DoubleSupplier strafeSupplier,
      DoubleSupplier rotationSupplier,
      CommandSwerveDrivetrain swerve) {

    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotationSupplier = rotationSupplier;
    this.swerve = swerve;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double forwardSpeed = -forwardSupplier.getAsDouble() * SwerveConstants.maxTranslationalSpeed;
    double strafeSpeed = -strafeSupplier.getAsDouble() * SwerveConstants.maxTranslationalSpeed;
    double rotationSpeed = -rotationSupplier.getAsDouble() * SwerveConstants.maxRotationalSpeed;

    // Create double variables for forward, strafe, and rotation speed
    // Equals -supplier.getDouble() * SwerveConstants.maxTranslationalSpeed.in(MetersPerSecond);

    forwardSpeed = forwardRateLimiter.calculate(forwardSpeed);
    strafeSpeed = strafeRateLimiter.calculate(strafeSpeed);
    rotationSpeed = rotationRateLimiter.calculate(rotationSpeed);
    // Create forward, strafe, and rotation speed
    // Equal to __limiter.calculate(___Speed)

    // if speed is less than 1, set the speeds to zero and reset the value to zero.
    // inches to metesr for forward and strafe, and degrees to rotation for rotation.
    // Math.hypot for forward and strafe speed, and absolute value for rotational speed

    if (Math.hypot(forwardSpeed, strafeSpeed) <= Units.inchesToMeters(1)) {
      forwardSpeed = 0;
      strafeSpeed = 0;

      forwardRateLimiter.reset(0);
      strafeRateLimiter.reset(0);
    }

    if (Math.abs(rotationSpeed) <= Units.degreesToRadians(1)) {
      rotationSpeed = 0;

      rotationRateLimiter.reset(0);
    }

    swerve.setControl(
        fieldOriented
            .withVelocityX(forwardSpeed)
            .withVelocityY(strafeSpeed)
            .withRotationalRate(Units.rotationsToRadians(rotationSpeed)));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Reset limiters to zero
    // setControl for swerve
    // fieldoriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0)

    forwardRateLimiter.reset(0);
    strafeRateLimiter.reset(0);
    rotationRateLimiter.reset(0);
    fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
