
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
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Launch extends SubsystemBase {
  private final SparkMax m_Launch;
  private SparkMaxConfig intakeMotorConfig;

  /** Creates a new Launch. */
  public Launch() {
    m_Launch = new SparkMax(Constants.FuelSystemConstants.LAUNCH_MOTOR_ID, MotorType.kBrushless);

    intakeMotorConfig = new SparkMaxConfig();

    configureIntakeMotor(m_Launch, intakeMotorConfig);
  }

  private void configureIntakeMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
    
  }

  public RunCommand LaunchClose() {
    return new RunCommand(() -> {
      m_Launch.set(0.75);
    }, this);
    }

  public RunCommand LaunchFar() {
    return new RunCommand(() -> {
      m_Launch.set(1.0);
    }, this);
    }


  public RunCommand LaunchCollect() {
    return new RunCommand(() -> {
      m_Launch.set(0.25);
    }, this);
    }

  public RunCommand LaunchReverse() {
    return new RunCommand(() -> {
      m_Launch.set(-0.5);
    }, this);
    }

  public Command LaunchStop() {
    return new InstantCommand(() -> {
      m_Launch.set(0.0);
    }, this);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
