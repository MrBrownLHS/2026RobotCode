package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Constants;
import frc.robot.utilities.Dashboard;

public class RearIntakeLift extends SubsystemBase {

  public enum State {
    STORED,
    LIFTED,
    EXTENDED,
    MANUAL,
    SHUFFLE,
    IDLE
  }

  private State currentState = State.STORED;

  private final SparkMax m_RearIntakeLift;
  private final RelativeEncoder encoder;

  // Shuffle state
  private boolean shuffleInitialized = false;
  private boolean shuffleGoingUp = true;
  private boolean shufflePaused = false;
  private double shuffleTarget = 0.0;
  private final Timer shufflePauseTimer = new Timer();

  // Tunable values
  private final double tolerance = 0.25;
  private final double slowZone = 5.0;

  // Normal motion speeds
  private final double fastUpSpeed = 0.15;
  private final double slowUpSpeed = 0.08;
  private final double fastDownSpeed = -0.15;
  private final double slowDownSpeed = -0.08;

  // Shuffle motion speeds
  private final double fastShuffleUpSpeed = 0.12;
  private final double slowShuffleUpSpeed = 0.07;
  private final double fastShuffleDownSpeed = -0.12;
  private final double slowShuffleDownSpeed = -0.07;

  private final double shufflePauseSeconds = 0.10;

  private double manualOutput = 0.0;

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
        shuffleInitialized = false;
        shuffleGoingUp = true;
        shufflePaused = false;
        shuffleTarget = Constants.FuelSystemConstants.REAR_INTAKE_LIFT_STORED_POSITION;
        shufflePauseTimer.stop();
        shufflePauseTimer.reset();
      }
    }
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

      case SHUFFLE:
        return shuffleTarget;

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
    return Math.abs(
        getPosition() - Constants.FuelSystemConstants.REAR_INTAKE_LIFT_EXTEND_POSITION) < tolerance;
  }

  public boolean isStored() {
    return Math.abs(
        getPosition() - Constants.FuelSystemConstants.REAR_INTAKE_LIFT_STORED_POSITION) < tolerance;
  }

  public boolean isLifted() {
    return Math.abs(
        getPosition() - Constants.FuelSystemConstants.REAR_INTAKE_LIFT_LIFTED_POSITION) < tolerance;
  }

  public void stop() {
    currentState = State.IDLE;
    m_RearIntakeLift.set(0.0);
  }

  public void manualControl(double speed) {
    currentState = State.MANUAL;
    manualOutput = speed;
  }

  /* =============================
     HELPERS
     ============================= */

  private double moveTowardTarget(
      double target,
      double fastPositiveSpeed,
      double slowPositiveSpeed,
      double fastNegativeSpeed,
      double slowNegativeSpeed) {

    double current = getPosition();
    double error = target - current;
    double absError = Math.abs(error);

    if (absError < tolerance) {
      return 0.0;
    }

    boolean useSlowSpeed = absError < slowZone;

    if (error > 0) {
      return useSlowSpeed ? slowPositiveSpeed : fastPositiveSpeed;
    } else {
      return useSlowSpeed ? slowNegativeSpeed : fastNegativeSpeed;
    }
  }

  /* =============================
     PERIODIC CONTROL
     ============================= */

  @Override
  public void periodic() {

    double output = 0.0;
    double current = getPosition();

    double stored = Constants.FuelSystemConstants.REAR_INTAKE_LIFT_STORED_POSITION;
    double lifted = Constants.FuelSystemConstants.REAR_INTAKE_LIFT_LIFTED_POSITION;
    double extended = Constants.FuelSystemConstants.REAR_INTAKE_LIFT_EXTEND_POSITION;

    switch (currentState) {

      case IDLE:
        output = 0.0;
        break;

      case MANUAL:
        output = manualOutput;
        break;

      case STORED:
        output = moveTowardTarget(
            stored,
            fastUpSpeed, slowUpSpeed,
            fastDownSpeed, slowDownSpeed);
        break;

      case LIFTED:
        output = moveTowardTarget(
            lifted,
            fastUpSpeed, slowUpSpeed,
            fastDownSpeed, slowDownSpeed);
        break;

      case EXTENDED:
        output = moveTowardTarget(
            extended,
            fastUpSpeed, slowUpSpeed,
            fastDownSpeed, slowDownSpeed);
        break;

      case SHUFFLE:

        // Step 1: always return to STORED before beginning shuffle
        if (!shuffleInitialized) {
          shuffleTarget = stored;

          if (Math.abs(current - stored) > tolerance) {
            output = moveTowardTarget(
                stored,
                fastUpSpeed, slowUpSpeed,
                fastDownSpeed, slowDownSpeed);
            break;
          } else {
            shuffleInitialized = true;
            shuffleGoingUp = true;
            shuffleTarget = lifted;
            shufflePaused = false;
            shufflePauseTimer.stop();
            shufflePauseTimer.reset();
          }
        }

        // Step 2: pause briefly at endpoints before reversing
        if (shufflePaused) {
          output = 0.0;

          if (shufflePauseTimer.hasElapsed(shufflePauseSeconds)) {
            shufflePaused = false;
            shufflePauseTimer.stop();
            shufflePauseTimer.reset();

            shuffleGoingUp = !shuffleGoingUp;
            shuffleTarget = shuffleGoingUp ? lifted : stored;
          }
          break;
        }

        // Step 3: if target reached, start pause
        if (Math.abs(current - shuffleTarget) < tolerance) {
          output = 0.0;
          shufflePaused = true;
          shufflePauseTimer.restart();
          break;
        }

        // Step 4: move toward shuffle target using gentler speeds
        output = moveTowardTarget(
            shuffleTarget,
            fastShuffleUpSpeed, slowShuffleUpSpeed,
            fastShuffleDownSpeed, slowShuffleDownSpeed);
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
    Dashboard.logBoolean("RearIntakeLift Shuffle Paused", () -> shufflePaused);
  }
}