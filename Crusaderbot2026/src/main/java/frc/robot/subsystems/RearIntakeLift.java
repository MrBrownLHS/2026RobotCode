package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Constants;
import frc.robot.utilities.Dashboard;

public class RearIntakeLift extends SubsystemBase {

  public enum State {
    STORED,
    LIFTED,
    EXTENDED,
    MANUAL,
    IDLE
  }

  private State currentState = State.STORED;

  private final SparkMax m_RearIntakeLift;
  private final RelativeEncoder encoder;

  // --- Tunable values ---

  private final double tolerance = 0.2;

  public RearIntakeLift() {

    m_RearIntakeLift = new SparkMax(
        Constants.FuelSystemConstants.REAR_INTAKE_LIFT_MOTOR_ID,
        MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.inverted(true);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);

    m_RearIntakeLift.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    encoder = m_RearIntakeLift.getEncoder();

    // Assume starting at stored
    encoder.setPosition(0.0);

    m_RearIntakeLift.set(0.0);
  }

  /* =============================
     STATE CONTROL
     ============================= */

  public void setState(State newState) {
    currentState = newState;
  }

  public State getState() {
    return currentState;
  }

  /* =============================
     POSITION LOGIC
     ============================= */

  private double getTargetPosition() {
    switch (currentState) {
      case STORED:
        return Constants.FuelSystemConstants.REAR_INTAKE_LIFT_STORED_POSITION;

      case LIFTED:
        return Constants.FuelSystemConstants.REAR_INTAKE_LIFT_LIFTED_POSITION;

      case EXTENDED:
        return Constants.FuelSystemConstants.REAR_INTAKE_LIFT_EXTEND_POSITION;

      default:
        return 0.0;
    }
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void zeroEncoder() {
    encoder.setPosition(0.0);
  }

  public boolean atSetpoint() {
    return Math.abs(getTargetPosition() - getPosition()) < tolerance;
  }

  public boolean isExtended() {
    return Math.abs(getPosition() - Constants.FuelSystemConstants.REAR_INTAKE_LIFT_EXTEND_POSITION) < tolerance;
  }

  public void stop() {
    m_RearIntakeLift.set(0.0);
  }

  private double manualOutput = 0.0;

  public void manualControl(double speed) {
    currentState = State.MANUAL;
    manualOutput = speed;
  }

  /* =============================
     PERIODIC CONTROL
     ============================= */

  @Override
  public void periodic() {

     double output = 0.0;

  switch (currentState) {

    case IDLE:
      output = 0.0;
      break;

    case MANUAL:
      output = manualOutput;
      break;

    case STORED:
    case LIFTED:
    case EXTENDED:

      double target = getTargetPosition();
      double current = getPosition();

      if (!atSetpoint()) {
        if (current < target) {
          output = 0.15;
        } else {
          output = -0.15;
        }
      } else {
        output = 0.0;
      }

      break;
  }

  m_RearIntakeLift.set(output);

    /* =============================
       DASHBOARD
       ============================= */
    Dashboard.logNumber("RearIntakeLift Position", this::getPosition);
    Dashboard.logNumber("RearIntakeLift Target", this::getTargetPosition);
    Dashboard.logString("RearIntakeLift State", () -> currentState.toString());
    Dashboard.logBoolean("RearIntakeLift At Target", this::atSetpoint);
  }
}