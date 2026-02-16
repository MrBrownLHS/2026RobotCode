// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.utilities.Constants;

public class ClimberWinch extends SubsystemBase {
  private final SparkMax m_ClimberWinch;
  private SparkMaxConfig climberWinchMotorConfig;
  /** Creates a new ClimberWinch. */
  public ClimberWinch() {
    m_ClimberWinch = new SparkMax(Constants.ClimberConstants.CLIMBER_WINCH_MOTOR_ID, MotorType.kBrushless);
    climberWinchMotorConfig = new SparkMaxConfig();
    configureClimberWinchMotor(m_ClimberWinch, climberWinchMotorConfig);
  }

  private void configureClimberWinchMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
  }
  
   public RunCommand ClimberWinchRun() {
     return new RunCommand(() -> {
       m_ClimberWinch.set(0.5);
     }, this);
     }

    public RunCommand ClimberWinchReverse() {
      return new RunCommand(() -> {
        m_ClimberWinch.set(-0.5);
      }, this);
      }

    public Command ClimberWinchStop() {
      return new RunCommand(() -> {
        m_ClimberWinch.set(0);
      }, this);
      }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
