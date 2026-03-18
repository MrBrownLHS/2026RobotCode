
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase;
import frc.robot.utilities.Dashboard;



public class RearIntake extends SubsystemBase {

    public enum State {
      IDLE,
      INTAKE_COLLECT,
      INTAKE_LAUNCH,
      INTAKE_REVERSE,
    }

  private State currentState = State.IDLE;

  private final SparkMax m_RearIntake;

 
  
 
  /** Creates a new Intake. */
  public RearIntake() {
    m_RearIntake = new SparkMax(Constants.FuelSystemConstants.REARINTAKE_MOTOR_ID, MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
    
    m_RearIntake.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);


  }

  public void setState(State newState) {
    currentState = newState;
  }

  public State getState() {
    return currentState;
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentState) {
      case IDLE:
        m_RearIntake.set(0.0);
        break;
      case INTAKE_COLLECT:
        m_RearIntake.set(Constants.FuelSystemConstants.REARINTAKE_MOTOR_COLLECT_SPEED);
        break;
      case INTAKE_REVERSE:
        m_RearIntake.set(Constants.FuelSystemConstants.REARINTAKE_MOTOR_REVERSE_SPEED);
        break;
      }

    Dashboard.logString("Rear Intake State", () -> currentState.toString());
    
  }
}
