// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private LaserCan outakeLaser;

  private TalonFX indexerMotor;

  public Indexer() {
    outakeLaser = new LaserCan(14);
    indexerMotor = new TalonFX(2, "Cannie");
  }

  public Command runIndexer() {
    return run(this::index);
  }

  public Command stop() {
    return runOnce(this::stopIndexer);
  }

  // public void index() {
  //   if (!outakeLaserBroken()) {
  //     indexerMotor.set(IntakeConstants.indexerMotorSpeed);
  //   } else {
  //     stopIndexer();
  //   }
  // }
  public void index() {
    indexerMotor.set(IntakeConstants.indexerMotorSpeed);
  }

  public void stopIndexer() {
    indexerMotor.set(0.0);
  }

  public boolean outakeLaserBroken() {
    LaserCan.Measurement measurement = outakeLaser.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      // System.out.println("The target is " + measurement.distance_mm + "mm away!");
      // if (measurement.distance_mm < 500) {
      //   return true;
      // } else {
      //   return false;
      // }
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
