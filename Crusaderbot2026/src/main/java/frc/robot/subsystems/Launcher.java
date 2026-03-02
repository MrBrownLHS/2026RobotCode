
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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;



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

 
  private State currentState = State.IDLE;


  private final SparkMax m_Launch;

  private final RelativeEncoder launchEncoder;
 
  private final SparkClosedLoopController launchController;

  // Target RPM values for each launcher mode (tune these on the robot)
  private double launchFarSpeed = Constants.FuelSystemConstants.LAUNCH_FAR_RPM; // Tune
  private double launchCloseSpeed = Constants.FuelSystemConstants.LAUNCH_CLOSE_RPM; // Tune
  private double launchCollectSpeed = Constants.FuelSystemConstants.LAUNCH_COLLECT_RPM; // Tune
  private double launchRPMTolerance = Constants.FuelSystemConstants.LAUNCH_RPM_TOLERANCE; // Tune

  private final ShuffleboardTab fuelSystemTab = Shuffleboard.getTab("Fuel System");

  /**
   * Constructor: create motor/controller objects and apply configuration.
   */
  public Launcher() {
    // Create the SparkMax on the ID provided in Constants
    m_Launch = new SparkMax(Constants.FuelSystemConstants.LAUNCH_MOTOR_1_ID, MotorType.kBrushless);

    // Build a configuration object and set motor behavior
    SparkMaxConfig launchMotorConfig = new SparkMaxConfig();
    launchMotorConfig.idleMode(IdleMode.kCoast);
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

    // Grab the encoder and the closed-loop controller interfaces
    launchEncoder = m_Launch.getEncoder();
    launchController = m_Launch.getClosedLoopController();

    fuelSystemTab.addString("Launcher State", () -> currentState.toString());
    fuelSystemTab.addNumber("Launcher Target RPM", this::getTargetRPM);
    fuelSystemTab.addNumber("Launcher Actual RPM", launchEncoder::getVelocity);

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
    return Math.abs(getTargetRPM() - launchEncoder.getVelocity()) < launchRPMTolerance;
  }

  // Map the current state to a target RPM (used by periodic())
  private double getTargetRPM() {
    switch (currentState) {
      case LAUNCH_FAR:
        return launchFarSpeed;
      case LAUNCH_CLOSE:
        return launchCloseSpeed;
      case LAUNCH_COLLECT:
        return launchCollectSpeed;
      default:
        return 0.0;
    }
  }

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
  }
}
