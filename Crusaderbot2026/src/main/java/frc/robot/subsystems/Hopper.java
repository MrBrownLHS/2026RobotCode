// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Constants;

/**
 * Subsystem that controls the hopper extension actuator and reads two
 * limit switches (in/out). Provides simple methods to extend, retract and
 * stop the actuator as well as query the limit switches.
 */
public class Hopper extends SubsystemBase {

    public enum State {
      IDLE,
      EXTENDING,
      RETRACTING
    }
  
  private State currentState = State.IDLE;

  private final SparkMax m_HopperMotor;
  private final SparkMaxConfig hopperMotorConfig;

  // Limit switches wired to DIO ports
  private final DigitalInput limitIn;
  private final DigitalInput limitOut;

  public Hopper() {
    // Motor and DIO IDs come from Constants (update to match wiring)
    m_HopperMotor = new SparkMax(Constants.FuelSystemConstants.HOPPER_MOTOR_ID, MotorType.kBrushless);
    
    hopperMotorConfig = new SparkMaxConfig();
    configureHopperMotor(m_HopperMotor, hopperMotorConfig);
  
    limitIn = new DigitalInput(Constants.FuelSystemConstants.HOPPER_LIMIT_IN_DIO);
    limitOut = new DigitalInput(Constants.FuelSystemConstants.HOPPER_LIMIT_OUT_DIO);
  }

  private void configureHopperMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
  }



  public void setState(State newState) {

    if (newState == State.EXTENDING && isHopperOutLimitPressed())
        return;

    if (newState == State.RETRACTING && isHopperInLimitPressed())
        return;

    currentState = newState;
}

  public State getState() {
    return currentState;
  }

  /**
   * Returns true when the "in" limit switch is pressed (hopper fully retracted).
   * Note: wiring may be normally-closed or -open; invert the boolean if needed.
   */
  public boolean isHopperInLimitPressed() {
    return !limitIn.get();
  }

  /**
   * Returns true when the "out" limit switch is pressed (hopper fully extended).
   */
  public boolean isHopperOutLimitPressed() {
    return !limitOut.get();
  }

  /**
   * Convenience: true when either limit switch is pressed.
   */
  public boolean isAnyHopperLimitPressed() {
    return isHopperInLimitPressed() || isHopperOutLimitPressed();
  }

  /** Stop the hopper motor immediately. */
  public void stop() {
    m_HopperMotor.set(0.0);
  }




  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentState) {
      case IDLE:
        m_HopperMotor.set(0.0);
        break;
      case EXTENDING:
        // If out limit is hit, stop extending and return to IDLE
        if (isHopperOutLimitPressed()) {
          currentState = State.IDLE;
          break;}
        m_HopperMotor.set(Constants.FuelSystemConstants.HOPPER_EXTEND_SPEED);
        break;
      case RETRACTING:
        // If in limit is hit, stop retracting and return to IDLE
        if (isHopperInLimitPressed()) {
          currentState = State.IDLE;
          break;
        }
        m_HopperMotor.set(Constants.FuelSystemConstants.HOPPER_RETRACT_SPEED);
        break;
      } 
      SmartDashboard.putString("Hopper State", currentState.toString());
      SmartDashboard.putBoolean("Hopper In Limit", isHopperInLimitPressed());
      SmartDashboard.putBoolean("Hopper Out Limit", isHopperOutLimitPressed());
  }
}
