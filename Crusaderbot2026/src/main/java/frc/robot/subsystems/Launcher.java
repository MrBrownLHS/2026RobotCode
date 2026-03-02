
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
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * Launcher subsystem - controls the flywheel launcher using a SparkMax motor.
 *
 * This subsystem uses the SparkMax built-in closed-loop velocity controller
 * (configured via SparkMaxConfig) to reach target RPMs for different launcher
 * states. The subsystem exposes a simple state machine (setState) that
 * periodic() reads each scheduler cycle and commands the motor accordingly.
 */
public class Launcher extends SubsystemBase {

  // High-level launcher modes used by higher-level coordinators (SuperSystem)
  public enum State {
    IDLE,         // Motor stopped
    LAUNCH_FAR,   // High RPM for long shots
    LAUNCH_CLOSE, // Lower RPM for close shots
    LAUNCH_COLLECT // Low RPM to help collect/tower feed
  }

  // Currently active state (starts IDLE)
  private State currentState = State.IDLE;

  // REV SparkMax motor controller for the launcher NEO
  private final SparkMax m_Launch;
  // Encoder attached to the SparkMax (REV RelativeEncoder)
  private final RelativeEncoder launchEncoder;
  // SparkMax configuration object where we set idle mode, current limits, and closed-loop gains
  private final SparkMaxConfig launchMotorConfig;
  // REV-provided closed-loop controller object used to call setReference(...)
  private final SparkClosedLoopController launchController;

  // Target RPM values for each launcher mode (tune these on the robot)
  private static final double LAUNCH_FAR_RPM = 3000; // Tune
  private static final double LAUNCH_CLOSE_RPM = 2000; // Tune
  private static final double LAUNCH_COLLECT_RPM = 1500; // Tune
  // How close (RPM) we consider "at speed"
  private static final double LAUNCH_RPM_TOLERANCE = 150; // Tune

  /**
   * Constructor: create motor/controller objects and apply configuration.
   */
  public Launcher() {
    // Create the SparkMax on the ID provided in Constants
    m_Launch = new SparkMax(Constants.FuelSystemConstants.LAUNCH_MOTOR_1_ID, MotorType.kBrushless);

    // Build a configuration object and set motor behavior
    launchMotorConfig = new SparkMaxConfig();
    // Flywheel should coast when not powered (less abrupt torque)
    launchMotorConfig.idleMode(IdleMode.kCoast);
    // Configure current limits and voltage compensation from Constants
    launchMotorConfig.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    launchMotorConfig.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    launchMotorConfig.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);

    // Configure the motor controller's closed-loop PID gains and output range
    launchMotorConfig.closedLoop
        .pid(
            Constants.FuelSystemConstants.LAUNCH_P,
            Constants.FuelSystemConstants.LAUNCH_I,
            Constants.FuelSystemConstants.LAUNCH_D)
        .outputRange(-1.0, 1.0);

    // Apply the configuration to the motor controller. The ResetMode/PersistMode
    // overloads are passed as null here to keep existing controller defaults.
    m_Launch.configure(
        launchMotorConfig,
        (com.revrobotics.spark.SparkBase.ResetMode) null,
        (com.revrobotics.spark.SparkBase.PersistMode) null);

    // Grab the encoder and the closed-loop controller interfaces
    launchEncoder = m_Launch.getEncoder();
    launchController = m_Launch.getClosedLoopController();

    // Ensure motor starts stopped
    m_Launch.set(0.0);
  }

  /**
   * Set the desired launcher state. Higher-level code (SuperSystem/Commands)
   * should call this to request a mode change.
   */
  public void setState(State state) {
    currentState = state;
  }

  /**
   * Return true when the launcher encoder is within the RPM tolerance of target.
   */
  public boolean atSpeed() {
    return Math.abs(getTargetRPM() - launchEncoder.getVelocity()) < LAUNCH_RPM_TOLERANCE;
  }

  // Map the current state to a target RPM (used by periodic())
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

  /**
   * Scheduler-driven periodic: command the motor based on the current state.
   */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    switch (currentState) {
      case IDLE:
        // Stop the motor when idle
        m_Launch.set(0.0);
        break;
      case LAUNCH_FAR:
      case LAUNCH_CLOSE:
      case LAUNCH_COLLECT:
        // Use the motor controller's built-in velocity PID. The REV library
        // uses RPM for encoder velocity on a NEO by default, so pass RPM here.
        double target = getTargetRPM();
        // Optional feedforward (units: volts). The constants LAUNCH_KS and LAUNCH_KV
        // are used here to compute an approximate feedforward in volts. Make sure
        // these constants are tuned and have the correct units for your REV API.
        double ff = Constants.FuelSystemConstants.LAUNCH_KS * Math.signum(target)
            + Constants.FuelSystemConstants.LAUNCH_KV * target;
        // Call the non-deprecated overload that accepts an arbitrary feedforward.
        launchController.setReference(target, ControlType.kVelocity, ClosedLoopSlot.kSlot0, ff);
        break;
    }

    // Publish telemetry for tuning and debugging
    SmartDashboard.putString("Launcher State", currentState.toString());
    SmartDashboard.putNumber("Launcher RPM", launchEncoder.getVelocity());
  }
}
