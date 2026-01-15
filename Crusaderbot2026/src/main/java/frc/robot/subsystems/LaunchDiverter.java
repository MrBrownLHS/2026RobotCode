// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

public class LaunchDiverter extends SubsystemBase {
  private final SparkMax m_LaunchDiverter;
  private SparkMaxConfig launchDiverterMotorConfig;

  /** Creates a new LaunchDiverter. */
  public LaunchDiverter() {
    m_LaunchDiverter = new SparkMax(Constants.FuelSystemConstants.LAUNCH_DIVERTER_MOTOR_ID, MotorType.kBrushless);

    launchDiverterMotorConfig = new SparkMaxConfig();

    configureLaunchDiverterMotor(m_LaunchDiverter, launchDiverterMotorConfig);
  }

  private void configureLaunchDiverterMotor(SparkMax motor, SparkMaxConfig config){
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_550);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_550);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
    
  }

  public Command LaunchDivertertoCollect() {
    return new RunCommand(() -> {
      m_LaunchDiverter.set(0.75);
    }, this).withTimeout(1.0);
  }

  public Command LaunchDivertertoLaunch() {
    return new RunCommand(() -> {
      m_LaunchDiverter.set(-0.75);
    }, this).withTimeout(1.0);
  }

  public Command StopLaunchDiverter() {
    return new InstantCommand(() -> {
      m_LaunchDiverter.set(0.0);
    }, this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
