// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.util.GeomUtil;
import java.util.Arrays;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CoralAlign extends Command {
  public final Swerve drivetrain = TunerConstants.createDrivetrain();

  private PhotonTrackedTarget target;
  private Transform2d transformToTarget;
  private Pose2d aprilTagPose;

  private SwerveRequest.FieldCentric fieldOriented =
      new SwerveRequest.FieldCentric()
          .withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective)
          .withSteerRequestType(SteerRequestType.Position);

  private final ProfiledPIDController driveController =
      new ProfiledPIDController(
          0.15, 0.0, 0.0, new TrapezoidProfile.Constraints(15, 8), VisionConstants.loopPeriodSecs);
  private final ProfiledPIDController thetaController =
      new ProfiledPIDController(
          0.15, 0.0, 0.0, new TrapezoidProfile.Constraints(15, 8), VisionConstants.loopPeriodSecs);

  private final double ffMaxRadius = 0.8;
  private final double ffMinRadius = 0.2;
  private double driveErrorAbs;
  private double thetaErrorAbs;

  private Translation2d lastSetpointTranslation;

  private String offset;
  private Transform2d aprilTagOffset;

  private boolean running = false;

  /** Creates a new LeftCoralAlign. */
  public CoralAlign(String offset) {
    this.offset = offset;

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lastSetpointTranslation = drivetrain.getState().Pose.getTranslation();

    target = drivetrain.getBestAprilTag();
    int bestTarget = target.fiducialId;

    boolean found = Arrays.stream(VisionConstants.reefAprilTags).anyMatch(x -> x == bestTarget);

    if (!found) {
      cancel();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (offset == "Left") {
      aprilTagOffset = new Transform2d(0, VisionConstants.aprilTagReefOffset, new Rotation2d(0));
    }
    if (offset == "Right") {
      aprilTagOffset = new Transform2d(0, -VisionConstants.aprilTagReefOffset, new Rotation2d(0));
    }

    Transform3d transformToTarget3D = target.bestCameraToTarget;
    transformToTarget3D = transformToTarget3D.plus(VisionConstants.limelightTransform);
    double xTransform = transformToTarget3D.getX();
    double yTransform = transformToTarget3D.getY();
    Rotation2d rotation = transformToTarget3D.getRotation().toRotation2d();
    transformToTarget = new Transform2d(xTransform, yTransform, rotation);
    transformToTarget = transformToTarget.plus(aprilTagOffset);

    Pose2d robotPose = drivetrain.getState().Pose;
    aprilTagPose = robotPose.transformBy(transformToTarget);

    // Gets the speed of the drive
    double currentDistance = robotPose.getTranslation().getDistance(aprilTagPose.getTranslation());
    double ffScaler =
        MathUtil.clamp((currentDistance - ffMinRadius) / (ffMaxRadius - ffMinRadius), 0.0, 1.0);
    driveErrorAbs = currentDistance;
    driveController.reset(
        lastSetpointTranslation.getDistance(aprilTagPose.getTranslation()),
        driveController.getSetpoint().velocity);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(driveErrorAbs, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) driveVelocityScalar = 0.0;
    lastSetpointTranslation =
        new Pose2d(
                aprilTagPose.getTranslation(),
                robotPose.getTranslation().minus(aprilTagPose.getTranslation()).getAngle())
            .transformBy(
                GeomUtil.translationToTransform(driveController.getSetpoint().position, 0.0))
            .getTranslation();

    double thetaVelocity =
        thetaController.getSetpoint().velocity * ffScaler
            + thetaController.calculate(
                robotPose.getRotation().getRadians(), aprilTagPose.getRotation().getRadians());
    thetaErrorAbs =
        Math.abs(robotPose.getRotation().minus(aprilTagPose.getRotation()).getRadians());
    if (thetaErrorAbs < thetaController.getPositionTolerance()) thetaVelocity = 0.0;

    var driveVelocity =
        new Pose2d(
                new Translation2d(),
                robotPose.getTranslation().minus(aprilTagPose.getTranslation()).getAngle())
            .transformBy(GeomUtil.translationToTransform(driveVelocityScalar, 0.0))
            .getTranslation();

    drivetrain.setControl(
        fieldOriented
            .withVelocityX(driveVelocity.getX())
            .withVelocityY(driveVelocity.getY())
            .withRotationalRate(thetaVelocity));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    running = false;
    drivetrain.setControl(fieldOriented.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  public boolean atGoal() {
    return running && driveController.atGoal() && thetaController.atGoal();
  }

  /** Checks if the robot pose is within the allowed drive and theta tolerances. */
  public boolean withinTolerance(double driveTolerance, Rotation2d thetaTolerance) {
    return running
        && Math.abs(driveErrorAbs) < driveTolerance
        && Math.abs(thetaErrorAbs) < thetaTolerance.getRadians();
  }

  /** Returns whether the command is actively running. */
  public boolean isRunning() {
    return running;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
