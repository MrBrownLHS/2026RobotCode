// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.utilities.Constants;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;


public class Aim extends SubsystemBase {
  /** Creates a new Aim. */
  private periodicIO aimPeriodicIO;
  private static Aim aimInstance;
  public static Aim getInstance() {
    if (aimInstance == null) {
      aimInstance = new Aim();
    }
    return aimInstance;
  }

  private SparkMax m_AimMotor;
  private RelativeEncoder m_AimEncoder;
  private SparkClosedLoopController m_AimController;

  private TrapezoidProfile aimProfile;
  private TrapezoidProfile.State aimCurrentState = new TrapezoidProfile.State();
  private TrapezoidProfile.State aimGoalState = new TrapezoidProfile.State();
  private double prevUpdateTime = Timer.getFPGATimestamp();

  private Aim() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
