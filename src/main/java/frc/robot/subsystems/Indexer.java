// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {

  /** Creates a new Indexer. */
  LaserCan intakeLaser;

  LaserCan outakeLaser;

  TalonFX indexerMotor = new TalonFX(0, "Cannie");

  public Indexer() {
    outakeLaser = new LaserCan(14);
    intakeLaser = new LaserCan(15);
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

  public void index() {
    if (outakeLaserBroken()) {
      indexerMotor.set(.2);
    } else {
      stopIndexer();
    }
  }

  public void stopIndexer() {
    indexerMotor.set(0.0);
  }

  public boolean intakeLaserBroken() {
    LaserCan.Measurement measurement = intakeLaser.getMeasurement();
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
