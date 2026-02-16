// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utilities.Constants;

/**
 * Subsystem that controls the hopper extension actuator and reads two
 * limit switches (in/out). Provides simple methods to extend, retract and
 * stop the actuator as well as query the limit switches.
 */
public class Hopper extends SubsystemBase {
  private final SparkMax m_HopperMotor;
  private final SparkMaxConfig motorConfig;

  // Limit switches wired to DIO ports
  private final DigitalInput limitIn;
  private final DigitalInput limitOut;

  public Hopper() {
    // Motor and DIO IDs come from Constants (update to match wiring)
    m_HopperMotor = new SparkMax(Constants.FuelSystemConstants.HOPPER_MOTOR_ID, MotorType.kBrushless);
    motorConfig = new SparkMaxConfig();
    motorConfig.idleMode(IdleMode.kBrake);
  
    limitIn = new DigitalInput(Constants.FuelSystemConstants.HOPPER_LIMIT_IN_DIO);
    limitOut = new DigitalInput(Constants.FuelSystemConstants.HOPPER_LIMIT_OUT_DIO);
  }

  @Override
  public void periodic() {
    // No periodic work required; subsystem provides control methods used by commands
  }

  /** Extend the hopper at the supplied percent output (positive extends). */
  public void extend(double percent) {
    m_HopperMotor.set(Math.abs(percent));
  }

  /** Retract the hopper at the supplied percent output (positive retracts). */
  public void retract(double percent) {
    m_HopperMotor.set(-Math.abs(percent));
  }

  /** Stop the hopper motor immediately. */
  public void stop() {
    m_HopperMotor.set(0.0);
  }

  /**
   * Returns true when the "in" limit switch is pressed (hopper fully retracted).
   * Note: depending on wiring, get() may return false when pressed; adjust as needed.
   */
  public boolean isHopperInLimitPressed() {
    return limitIn.get();
  }

  /**
   * Returns true when the "out" limit switch is pressed (hopper fully extended).
   */
  public boolean isHopperOutLimitPressed() {
    return limitOut.get();
  }

  /**
   * Factory command: retract hopper until the "in" limit switch is pressed.
   */
  public Command HopperIn() {
    return new RunCommand(() -> retract(Constants.FuelSystemConstants.HOPPER_RETRACT_SPEED), this)
        .until(() -> isHopperInLimitPressed());
  }

  /**
   * Factory command: extend hopper until the "out" limit switch is pressed.
   */
  public Command HopperOut() {
    return new RunCommand(() -> extend(Constants.FuelSystemConstants.HOPPER_EXTEND_SPEED), this)
        .until(() -> isHopperOutLimitPressed());
  }

  public Command HopperStop() {
    return new RunCommand(() -> stop(), this);
  } 
}
