// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private TalonFX elevatorMainMotor;

  private TalonFX elevatorFollowerMotor;
  private Follower follower;
  private StatusSignal<Angle> currentPosition;

  private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
  private DigitalInput buttonSwitch = new DigitalInput(30);
  private boolean isZeroed = false;
  private boolean isLimitConfigApplied = false;
  private Alert elevatorAlerts;

  public Elevator() {
    elevatorMainMotor = new TalonFX(ElevatorConstants.elevatorMainMotorID, "Cannie");
    elevatorFollowerMotor = new TalonFX(ElevatorConstants.elevatorFollowerMotorID, "Cannie");
    follower = new Follower(ElevatorConstants.elevatorMainMotorID, false);

    currentPosition = elevatorMainMotor.getPosition();
    // elevatorMainMotor.setPosition(Rotations.of(0));

    // currentPosition.setUpdateFrequency(50);

    elevatorFollowerMotor.setControl(follower);

    elevatorMainMotor.getConfigurator().apply(ElevatorConstants.elevatorConfigs);

    elevatorAlerts = new Alert("Elevator is not Zeroed!", AlertType.kWarning);
  }

  public boolean buttonPressed() {
    return buttonSwitch.get();
  }

  public Command homeElevator() {
    return run(() -> setSpeed(-0.1))
        .until(this::buttonPressed)
        .unless(this::buttonPressed)
        .finallyDo(this::stop);
  }

  public void stopElevator() {
    elevatorMainMotor.set(0);
  }

  public Command stop() {
    return runOnce(this::stopElevator);
  }

  public void setSpeed(double speed) {
    elevatorMainMotor.set(speed);
  }

  public void printPosition() {
    SmartDashboard.putNumber(
        "Elevator Position", elevatorMainMotor.getPosition().getValueAsDouble());
  }

  public Command moveToPosition(double height) {
    return run(() -> elevatorMainMotor.setControl(motionMagicRequest.withPosition(height)));
  }

  public Command downPosition() {
    return moveToPosition(0.0).onlyIf(() -> isZeroed);
  }

  @Override
  public void periodic() {

    elevatorAlerts.set(!isZeroed);

    printPosition();

    if (buttonPressed()) {
      elevatorMainMotor.setPosition(0);
      isZeroed = true;
    }

    if (isZeroed && !isLimitConfigApplied) {
      elevatorMainMotor
          .getConfigurator()
          .apply(
              ElevatorConstants.elevatorConfigs.withSoftwareLimitSwitch(
                  ElevatorConstants.limitSwitchConfigs));
      isLimitConfigApplied = true;
    }
  }
}
