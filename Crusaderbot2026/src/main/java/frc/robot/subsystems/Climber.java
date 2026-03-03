// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Constants;

public class Climber extends SubsystemBase {

  /** Simple state machine for the climber. */
  public enum State {
    IDLE,
    EXTENDING, // extend the climber arm/reach
    CLIMBING   // winch to lift the robot
  }

  private State currentState = State.IDLE;

  // Motor controllers for the climber (two motors: extend and winch)
  private final SparkMax leftWinchMotor;
  private final SparkMax rightWinchMotor;

  // Encoders for position feedback
  private final RelativeEncoder leftWinchEncoder;
  private final RelativeEncoder rightWinchEncoder;

  // Per-side home limit switches (true when pressed). Polarity may need
  // inversion depending on wiring (NC vs NO) — change logic if you see inverted behavior.
  private final DigitalInput leftHome = new DigitalInput(Constants.ClimberConstants.CLIMBER_LEFT_HOME_DIO);
  private final DigitalInput rightHome = new DigitalInput(Constants.ClimberConstants.CLIMBER_RIGHT_HOME_DIO);

  private final ShuffleboardTab climberTab = Shuffleboard.getTab("Climber");
  
  /**
   * Create motors, encoders and apply basic configuration.
   */
  public Climber() {
    // Map motor IDs from Constants
    leftWinchMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_WINCH_RIGHT_MOTOR_ID, MotorType.kBrushless);
    rightWinchMotor = new SparkMax(Constants.ClimberConstants.CLIMBER_WINCH_LEFT_MOTOR_ID, MotorType.kBrushless);

    // Apply basic motor configuration (idle mode, current limits, voltage comp)
    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.idleMode(IdleMode.kBrake);
    leftConfig.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    leftConfig.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    leftConfig.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
    
    leftWinchMotor.configure(
        leftConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.idleMode(IdleMode.kBrake);
    rightConfig.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    rightConfig.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    rightConfig.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
    
    rightWinchMotor.configure(
        rightConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    // Encoders for feedback
    leftWinchEncoder = leftWinchMotor.getEncoder();
    rightWinchEncoder = rightWinchMotor.getEncoder();

    // Ensure motors start stopped
    leftWinchMotor.set(0.0);
    rightWinchMotor.set(0.0);

    climberTab.addString("Climber State", () -> currentState.toString());
    climberTab.addBoolean("Climber Left Home", this::isLeftHomePressed);
    climberTab.addBoolean("Climber Right Home", this::isRightHomePressed);
    climberTab.addNumber("Climber Left Pos", this::getLeftPosition);
    climberTab.addNumber("Climber Right Pos", this::getRightPosition);
    climberTab.addNumber("Climber Pos Avg", this::getAveragePosition);
    climberTab.addBoolean("Climber Fault/Mismatch",
        () -> Math.abs(getLeftPosition() - getRightPosition())
                > Constants.ClimberConstants.CLIMBER_MISMATCH_TOLERANCE);
  }

  /** Set the climber state. Call this from commands or higher-level coordinators. */
  public void setState(State newState) {
    currentState = newState;
  }

  /** Return the current climber state. */
  public State getState() {
    return currentState;
  }

  /** Returns true when the home limit is pressed. Adjust polarity if needed. */
  // Removed single home helper; using per-side helpers below.

  /** Returns true when the left side home limit is pressed. */
  public boolean isLeftHomePressed() {
    return leftHome.get();
  }

  /** Returns true when the right side home limit is pressed. */
  public boolean isRightHomePressed() {
    return rightHome.get();
  }

  /** Stop both climber motors immediately. */
  public void stop() {
    leftWinchMotor.set(0.0);
    rightWinchMotor.set(0.0);
  }

  /** Average encoder position (rotations) across both encoders. */
  private double getAveragePosition() {
    return (leftWinchEncoder.getPosition() + rightWinchEncoder.getPosition()) / 2.0;
  }

  /** Left encoder position (rotations). */
  private double getLeftPosition() {
    return leftWinchEncoder.getPosition();
  }

  /** Right encoder position (rotations). */
  private double getRightPosition() {
    return rightWinchEncoder.getPosition();
  }

  

  @Override
  public void periodic() {
    // Homing: if a home limit switch is pressed, zero that side's encoder so
    // software soft-limits are relative to the physical home.
    if (isLeftHomePressed()) {
      leftWinchEncoder.setPosition(0.0);
    }
    if (isRightHomePressed()) {
      rightWinchEncoder.setPosition(0.0);
    }

    // State machine: decide motor outputs based on the current state and soft limits.
    switch (currentState) {
      case IDLE:
        stop();
        break;

      case EXTENDING:
        // Extend each side independently while respecting per-side soft limits.
        boolean leftAtMax = getLeftPosition() >= Constants.ClimberConstants.CLIMBER_MAX_EXTENSION;
        boolean rightAtMax = getRightPosition() >= Constants.ClimberConstants.CLIMBER_MAX_EXTENSION;

        if (leftAtMax) {
          leftWinchMotor.set(0.0);
        } else {
          leftWinchMotor.set(Constants.ClimberConstants.CLIMBER_EXTEND_SPEED);
        }

        if (rightAtMax) {
          rightWinchMotor.set(0.0);
        } else {
          rightWinchMotor.set(Constants.ClimberConstants.CLIMBER_EXTEND_SPEED);
        }

        // If both sides are at their max, go back to IDLE
        if (leftAtMax && rightAtMax) {
          currentState = State.IDLE;
        }
        break;

      case CLIMBING:
        // Retract (climb) each side independently while respecting home/min limits.
        boolean leftAtMin = getLeftPosition() <= Constants.ClimberConstants.CLIMBER_MIN_EXTENSION || isLeftHomePressed();
        boolean rightAtMin = getRightPosition() <= Constants.ClimberConstants.CLIMBER_MIN_EXTENSION || isRightHomePressed();

        if (leftAtMin) {
          leftWinchMotor.set(0.0);
        } else {
          leftWinchMotor.set(Constants.ClimberConstants.CLIMBER_RETRACT_SPEED);
        }

        if (rightAtMin) {
          rightWinchMotor.set(0.0);
        } else {
          rightWinchMotor.set(Constants.ClimberConstants.CLIMBER_RETRACT_SPEED);
        }

        // If both sides are at their min/home, go back to IDLE
        if (leftAtMin && rightAtMin) {
          currentState = State.IDLE;
        }
        break;
    }

    // Mismatch detection: if one side moves significantly more than the other,
    // stop to avoid structural damage.
    double mismatch = Math.abs(getLeftPosition() - getRightPosition());
    if (mismatch > Constants.ClimberConstants.CLIMBER_MISMATCH_TOLERANCE) {
      stop();
      currentState = State.IDLE;
    }
  }
}
