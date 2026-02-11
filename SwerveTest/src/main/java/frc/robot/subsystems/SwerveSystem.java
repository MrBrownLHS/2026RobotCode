// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * SwerveSystem manages four swerve modules and a Pigeon IMU.
 * Uses REV CANSparkMax for motors and CTRE CANCoder for module absolute steering.
 */
public class SwerveSystem extends SubsystemBase {
  // CAN IDs - replace these with your real IDs
  private static final int PIGEON_ID = 1;

  private final SwerveModule frontLeft;
  private final SwerveModule frontRight;
  private final SwerveModule backLeft;
  private final SwerveModule backRight;

  private final PigeonIMU pigeon;

  public SwerveSystem() {
    // Example module CAN IDs and absolute encoder offsets (degrees)
    frontLeft = new SwerveModule(2, 3, 10.0); // driveId, steerId, cancoderOffset
    frontRight = new SwerveModule(4, 5, -5.0);
    backLeft = new SwerveModule(6, 7, 2.5);
    backRight = new SwerveModule(8, 9, -12.0);

    pigeon = new PigeonIMU(PIGEON_ID);
    zeroGyro();
  }

  /** Set desired state for each module. Length of states must be 4 (FL, FR, BL, BR). */
  public void setModuleStates(SwerveModuleState[] states) {
    if (states == null || states.length != 4) {
      throw new IllegalArgumentException("states must be length 4");
    }
    frontLeft.setDesiredState(states[0]);
    frontRight.setDesiredState(states[1]);
    backLeft.setDesiredState(states[2]);
    backRight.setDesiredState(states[3]);
  }

  public double getYaw() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr[0];
  }

  public void zeroGyro() {
    pigeon.setYaw(0.0);
  }

  @Override
  public void periodic() {
    // update module control loops
    frontLeft.periodic();
    frontRight.periodic();
    backLeft.periodic();
    backRight.periodic();
  }
}
