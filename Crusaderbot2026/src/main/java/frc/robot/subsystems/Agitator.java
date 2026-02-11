// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import frc.robot.utilities.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Agitator extends SubsystemBase {
    private final SparkMax m_Agitator;
    private SparkMaxConfig agitatorMotorConfig;

     /** Creates a new Agitator. */
     public Agitator() {
       m_Agitator = new SparkMax(Constants.FuelSystemConstants.AGITATOR_MOTOR_ID, MotorType.kBrushless);
 
       agitatorMotorConfig = new SparkMaxConfig();
 
       configureAgitatorMotor(m_Agitator, agitatorMotorConfig);
     }
 
     private void configureAgitatorMotor(SparkMax motor, SparkMaxConfig config){
       config.idleMode(IdleMode.kBrake);
       config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
       config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
       config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
       
     }
 
     public RunCommand AgitatorRun() {
       return new RunCommand(() -> {
         m_Agitator.set(0.5);
       }, this);
       }
 
 
     public RunCommand AgitatorReverse() {
       return new RunCommand(() -> {
         m_Agitator.set(-0.5);
       }, this);
       }

     public Command AgitatorStop() {
       return new RunCommand(() -> m_Agitator.set(0.0), this);
     }
 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
