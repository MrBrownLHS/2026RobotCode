// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.Constants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Agitator extends SubsystemBase {

    public enum State {
      IDLE,
      AGITATE,
      REVERSE,
    }
    
    private State currentState = State.IDLE;
    
    private final SparkMax m_Agitator;
    

    private final ShuffleboardTab fuelSystemTab = Shuffleboard.getTab("Fuel System");

     /** Creates a new Agitator. */
     public Agitator() {
       m_Agitator = new SparkMax(
        Constants.FuelSystemConstants.AGITATOR_MOTOR_ID, MotorType.kBrushless);
      
       SparkMaxConfig config = new SparkMaxConfig();
       config.idleMode(IdleMode.kBrake);
       config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_550);
       config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_550);
       config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
       
       m_Agitator.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

      fuelSystemTab.addString("Agitator State", () -> currentState.toString());

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
        m_Agitator.set(0.0);
        break;
      case AGITATE:
        m_Agitator.set(Constants.FuelSystemConstants.AGITATOR_AGITATE_SPEED);
        break;
      case REVERSE:
        m_Agitator.set(Constants.FuelSystemConstants.AGITATOR_REVERSE_SPEED);
        break;
    }
  }
}
