package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Constants;
import frc.robot.utilities.Dashboard;

public class RearIntakeLift extends SubsystemBase {

  public enum State {
    STORED,
    LIFTED,
    EXTENDED,
    SHUFFLE,
    IDLE
  }

  private State currentState = State.STORED;

  private final SparkMax m_RearIntakeLift;
  private final RelativeEncoder encoder;

  // Tunable values
  private final double tolerance = 0.10;
  private final double upSpeed = Constants.FuelSystemConstants.REAR_INTAKE_LIFT_SPEED;
  private final double downSpeed = Constants.FuelSystemConstants.REAR_INTAKE_EXTEND_SPEED;

  // Shuffle state
  private boolean shuffleGoingUp = true;
  private boolean shufflePaused = false;
  private double shuffleTarget = -1.5;
  private final double shufflePauseSeconds = 0.5;
  private final Timer shufflePauseTimer = new Timer();

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
    if (newState != currentState) {
      currentState = newState;

      if (newState == State.SHUFFLE) {
        shuffleGoingUp = true;
        shufflePaused = false;
        shuffleTarget = Constants.FuelSystemConstants.REAR_INTAKE_LIFT_EXTEND_POSITION;
        shufflePauseTimer.stop();
        shufflePauseTimer.reset();
      }
    }
  }

  public State getState() {
    return currentState;
  }

  public void stop() {
    currentState = State.IDLE;
    m_RearIntakeLift.set(0.0);
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

      case SHUFFLE:
        return shuffleTarget;

      case IDLE:
      default:
        return getPosition();
    }
  }

  public double getPosition() {
    return encoder.getPosition();
  }

  public void zeroEncoder() {
    encoder.setPosition(0.0);
  }

  public boolean atSetpoint() {
    if (currentState == State.IDLE) {
      return true;
    }
    return Math.abs(getTargetPosition() - getPosition()) < tolerance;
  }

  public boolean isStored() {
    return Math.abs(
        getPosition() - Constants.FuelSystemConstants.REAR_INTAKE_LIFT_STORED_POSITION) < tolerance;
  }

  public boolean isLifted() {
    return Math.abs(
        getPosition() - Constants.FuelSystemConstants.REAR_INTAKE_LIFT_LIFTED_POSITION) < tolerance;
  }

  public boolean isExtended() {
    return Math.abs(
        getPosition() - Constants.FuelSystemConstants.REAR_INTAKE_LIFT_EXTEND_POSITION) < tolerance;
  }

  /* =============================
     HELPERS
     ============================= */

  private double moveTowardTarget(double target) {
    double current = getPosition();
    double error = target - current;

    if (Math.abs(error) < tolerance) {
      return 0.0;
    }

    return error > 0 ? upSpeed : downSpeed;
  }

  /* =============================
     PERIODIC CONTROL
     ============================= */

  @Override
  public void periodic() {
    double output = 0.0;

    double lifted = Constants.FuelSystemConstants.REAR_INTAKE_LIFT_LIFTED_POSITION;
    double extended = Constants.FuelSystemConstants.REAR_INTAKE_LIFT_EXTEND_POSITION;
    double current = getPosition();

    switch (currentState) {
      case IDLE:
        output = 0.0;
        break;

      case STORED:
      case LIFTED:
      case EXTENDED:
        output = moveTowardTarget(getTargetPosition());
        break;

      case SHUFFLE:
        if (shufflePaused) {
          output = 0.0;

          if (shufflePauseTimer.hasElapsed(shufflePauseSeconds)) {
            shufflePaused = false;
            shufflePauseTimer.stop();
            shufflePauseTimer.reset();

            shuffleGoingUp = !shuffleGoingUp;
            shuffleTarget = shuffleGoingUp ? extended : lifted;
          }
          break;
        }

        if (Math.abs(current - shuffleTarget) < tolerance) {
          output = 0.0;
          shufflePaused = true;
          shufflePauseTimer.restart();
          break;
        }

        output = moveTowardTarget(shuffleTarget);
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
    Dashboard.logBoolean("RearIntakeLift Stored", this::isStored);
    Dashboard.logBoolean("RearIntakeLift Lifted", this::isLifted);
    Dashboard.logBoolean("RearIntakeLift Extended", this::isExtended);
    Dashboard.logBoolean("RearIntakeLift Shuffle Paused", () -> shufflePaused);
  }
}