
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import frc.robot.utilities.Constants;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Launcher extends SubsystemBase {

  public enum State {
    IDLE,
    LAUNCH_FAR,
    LAUNCH_CLOSE,
    LAUNCH_COLLECT
  }

  private State currentState = State.IDLE;

  private final SparkMax m_Launch;
  private final RelativeEncoder launchEncoder;
  private final SparkMaxConfig launchMotorConfig;

  // WPILib PID controller to compute motor outputs to reach target RPM
  private final PIDController pidController;
  // Optional feedforward (left at zero unless tuned)
  private final SimpleMotorFeedforward feedforward;

  private static final double LAUNCH_FAR_RPM = 5200; // Tune
  private static final double LAUNCH_CLOSE_RPM = 4000; // Tune
  private static final double LAUNCH_COLLECT_RPM = 1500; // Tune
  private static final double LAUNCH_RPM_TOLERANCE = 150; // Tune

  /** Creates a new Launch. */
  public Launcher() {
  m_Launch = new SparkMax(Constants.FuelSystemConstants.LAUNCH_MOTOR_1_ID, MotorType.kBrushless);

  launchMotorConfig = new SparkMaxConfig();
  launchMotorConfig.idleMode(IdleMode.kBrake);
  launchMotorConfig.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
  launchMotorConfig.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
  launchMotorConfig.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);
  // Apply configuration to the motor (match pattern used in other subsystems)
 
  launchEncoder = m_Launch.getEncoder();

  // Initialize PID controller and optional feedforward using constants
  pidController = new PIDController(Constants.FuelSystemConstants.LAUNCH_P, Constants.FuelSystemConstants.LAUNCH_I, Constants.FuelSystemConstants.LAUNCH_D);
  pidController.setTolerance(LAUNCH_RPM_TOLERANCE);
  feedforward = new SimpleMotorFeedforward(Constants.FuelSystemConstants.LAUNCH_KS, Constants.FuelSystemConstants.LAUNCH_KV, 0.0);

  m_Launch.set(0.0);
  }

  public void setState(State state) {
    currentState = state;
  }

  public boolean atSpeed() {
    return pidController.atSetpoint();
  }

  private double getTargetRPM() {
    switch (currentState) {
      case LAUNCH_FAR:
        return LAUNCH_FAR_RPM;
      case LAUNCH_CLOSE:
        return LAUNCH_CLOSE_RPM;
      case LAUNCH_COLLECT:
        return LAUNCH_COLLECT_RPM;
      default:
        return 0.0;
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentState) {
      case IDLE:
        m_Launch.set(0.0);
        break;
      case LAUNCH_FAR:
      case LAUNCH_CLOSE:
      case LAUNCH_COLLECT:
        double target = getTargetRPM();
        double measurement = launchEncoder.getVelocity(); // RPM (RelativeEncoder.getVelocity() returns RPM by default)
        // Calculate PID output (tuned so output approximates -1..1 motor set)
        double pidOutput = pidController.calculate(measurement, target);
        //Optionally add feedforward (commented out until tuned):
        double ff = feedforward.calculate(target);
        double output = MathUtil.clamp(pidOutput + ff / 12.0, -1.0, 1.0);
        //double output = MathUtil.clamp(pidOutput, -1.0, 1.0);
        m_Launch.set(output);
        break;
    }
    SmartDashboard.putString("Launcher State", currentState.toString());
    SmartDashboard.putNumber("Launcher RPM", launchEncoder.getVelocity());
  }
}
