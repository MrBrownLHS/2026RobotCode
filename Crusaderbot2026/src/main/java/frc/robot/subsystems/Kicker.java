
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



public class Kicker extends SubsystemBase {
  private final SparkMax m_Kicker;
  private SparkMaxConfig kickerMotorConfig;
 
  /** Creates a new Index. */
  public Kicker() {
    m_Kicker = new SparkMax(Constants.FuelSystemConstants.KICKER_MOTOR_ID, MotorType.kBrushless);

    kickerMotorConfig = new SparkMaxConfig();

    configureKickerMotor(m_Kicker, kickerMotorConfig);
  }

  private void configureKickerMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
     
  }

  public RunCommand KickerCollect() {
    return new RunCommand(() -> {
      m_Kicker.set(0.15);
    }, this);
    }


  public RunCommand KickerCloseLaunch() {
    return new RunCommand(() -> {
      m_Kicker.set(0.75);
    }, this);
    }
    
  public RunCommand KickerFarLaunch() {
    return new RunCommand(() -> {
      m_Kicker.set(0.95);
    }, this);
    }
    
  public RunCommand KickerReverse() {
    return new RunCommand(() -> {
      m_Kicker.set(-0.25);
    }, this);
    }

  public Command KickerStop() {
    return new InstantCommand(() -> {
      m_Kicker.set(0.0);
    }, this);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
