package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import frc.robot.utilities.Constants;
import frc.robot.utilities.Dashboard;

public class Launcher extends SubsystemBase {

  public enum State {
    IDLE,
    LAUNCH_FAR,
    LAUNCH_CLOSE,
    LAUNCH_COLLECT
  }

  private State currentState = State.IDLE;
  private State lastState = null;

  // ===============================
  // Motors
  // ===============================

  private final SparkMax m_leftLauncher;   // NEW motor
  private final SparkMax m_rightLauncher;  // Existing motor

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  private final SparkClosedLoopController leftController;
  private final SparkClosedLoopController rightController;

  private double currentSetpointRPM = 0.0;
  private boolean isAtSpeedLatched = false;

  private final double rpmTolerance =
      Constants.FuelSystemConstants.LAUNCH_RPM_TOLERANCE;

  public Launcher() {

    m_leftLauncher = new SparkMax(
        Constants.FuelSystemConstants.LAUNCH_MOTOR_LEFT_ID,
        MotorType.kBrushless);

    m_rightLauncher = new SparkMax(
        Constants.FuelSystemConstants.LAUNCH_MOTOR_RIGHT_ID,
        MotorType.kBrushless);

    SparkMaxConfig rightConfig = new SparkMaxConfig();
    rightConfig.inverted(true);
    rightConfig.idleMode(IdleMode.kCoast);
    rightConfig.openLoopRampRate(0.2);
    rightConfig.closedLoopRampRate(0.3);

    rightConfig.closedLoop
        .pid(
            Constants.FuelSystemConstants.LAUNCH_P,
            Constants.FuelSystemConstants.LAUNCH_I,
            Constants.FuelSystemConstants.LAUNCH_D)
        .outputRange(-0.2, 1.0);
    
    m_rightLauncher.configure(
        rightConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);


    SparkMaxConfig leftConfig = new SparkMaxConfig();
    leftConfig.inverted(false);
    leftConfig.idleMode(IdleMode.kCoast);
    leftConfig.openLoopRampRate(0.2);
    leftConfig.closedLoopRampRate(0.3);

    leftConfig.closedLoop
        .pid(
            Constants.FuelSystemConstants.LAUNCH_P,
            Constants.FuelSystemConstants.LAUNCH_I,
            Constants.FuelSystemConstants.LAUNCH_D)
        .outputRange(-0.2, 1.0);

    m_leftLauncher.configure(
        leftConfig,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    

    leftEncoder = m_leftLauncher.getEncoder();
    rightEncoder = m_rightLauncher.getEncoder();

    leftController = m_leftLauncher.getClosedLoopController();
    rightController = m_rightLauncher.getClosedLoopController();
  }

  // ===============================
  // Public API
  // ===============================

  public void setState(State state) {
    currentState = state;
  }

  public boolean atSpeed() {
    return isAtSpeedLatched;
  }

  public double getTargetRPM() {
    switch (currentState) {
      case LAUNCH_FAR:
        return Constants.FuelSystemConstants.LAUNCH_FAR_RPM;
      case LAUNCH_CLOSE:
        return Constants.FuelSystemConstants.LAUNCH_CLOSE_RPM;
      case LAUNCH_COLLECT:
        return Constants.FuelSystemConstants.LAUNCH_COLLECT_RPM;
      default:
        return 0.0;
    }
  }

  // ===============================
  // Periodic
  // ===============================

  @Override
  public void periodic() {

    if (currentState != lastState) {

      double targetRPM = getTargetRPM();
      currentSetpointRPM = targetRPM;

      if (targetRPM == 0.0) {

        m_leftLauncher.set(0.0);
        m_rightLauncher.set(0.0);
        isAtSpeedLatched = false;

      } else {

        double ff =
            Constants.FuelSystemConstants.LAUNCH_KS * Math.signum(targetRPM)
            + Constants.FuelSystemConstants.LAUNCH_KV * targetRPM;

        leftController.setReference(
            targetRPM,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ff);

        rightController.setReference(
            targetRPM,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ff);

        isAtSpeedLatched = false;
      }

      lastState = currentState;
    }

    // ===============================
    // At Speed Logic (Both Wheels)
    // ===============================

    double leftError = currentSetpointRPM - leftEncoder.getVelocity();
    double rightError = currentSetpointRPM - rightEncoder.getVelocity();

    if (currentSetpointRPM > 0.0) {

      boolean leftAtSpeed = Math.abs(leftError) < rpmTolerance;
      boolean rightAtSpeed = Math.abs(rightError) < rpmTolerance;

      if (!isAtSpeedLatched && leftAtSpeed && rightAtSpeed) {
        isAtSpeedLatched = true;
      }

      if (isAtSpeedLatched &&
          (Math.abs(leftError) > rpmTolerance * 2.0 ||
           Math.abs(rightError) > rpmTolerance * 2.0)) {
        isAtSpeedLatched = false;
      }

    } else {
      isAtSpeedLatched = false;
    }

    // ===============================
    // Dashboard Logging
    // ===============================

    Dashboard.logBoolean("Launcher At Speed", () -> isAtSpeedLatched);
    Dashboard.logNumber("Launcher Target RPM", () -> currentSetpointRPM);

    Dashboard.logNumber("Launcher Left RPM", leftEncoder::getVelocity);
    Dashboard.logNumber("Launcher Right RPM", rightEncoder::getVelocity);

    Dashboard.logNumber("Launcher Left Error", () -> leftError);
    Dashboard.logNumber("Launcher Right Error", () -> rightError);

    Dashboard.logString("Launcher State", () -> currentState.toString());
  }
}