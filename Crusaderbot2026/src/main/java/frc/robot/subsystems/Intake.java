
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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



public class Intake extends SubsystemBase {
  private final SparkMax m_Intake;
  private SparkMaxConfig intakeMotorConfig;
 
  /** Creates a new Intake. */
  public Intake() {
    m_Intake = new SparkMax(Constants.FuelSystemConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);

    intakeMotorConfig = new SparkMaxConfig();

    configureIntakeMotor(m_Intake, intakeMotorConfig);
  }

  private void configureIntakeMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
    
  }

  public RunCommand IntakeCollect() {
    return new RunCommand(() -> {
      m_Intake.set(-0.5);
    }, this);
    }


  public RunCommand IntakeLaunch() {
    return new RunCommand(() -> {
      m_Intake.set(-0.75);
    }, this);
    }

  public RunCommand IntakeReverse() {
    return new RunCommand(() -> {
      m_Intake.set(0.5);
    }, this);
    }

  public Command IntakeStop() {
    return new RunCommand(() -> {
      m_Intake.set(0.0);
    }, this);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
