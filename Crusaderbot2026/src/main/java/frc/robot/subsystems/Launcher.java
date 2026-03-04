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

  private final SparkMax launcherMotor;
  private final RelativeEncoder launcherEncoder;
  private final SparkClosedLoopController launcherController;

  private double currentSetpointRPM = 0.0;
  private boolean isAtSpeedLatched = false;

  private final double rpmTolerance =
      Constants.FuelSystemConstants.LAUNCH_RPM_TOLERANCE;

  public Launcher() {

    launcherMotor = new SparkMax(
        Constants.FuelSystemConstants.LAUNCH_MOTOR_1_ID,
        MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.inverted(true);

    config.closedLoop
        .pid(
            Constants.FuelSystemConstants.LAUNCH_P,
            Constants.FuelSystemConstants.LAUNCH_I,
            Constants.FuelSystemConstants.LAUNCH_D)
        .outputRange(-1.0, 1.0);

    launcherMotor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    launcherEncoder = launcherMotor.getEncoder();
    launcherController = launcherMotor.getClosedLoopController();
  }

  public void setState(State state) {
    currentState = state;
  }

  public boolean atSpeed() {
    return isAtSpeedLatched;
  }

  private double getTargetRPM() {
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

  @Override
  public void periodic() {

    // Only update motor if state changed
    if (currentState != lastState) {

      double targetRPM = getTargetRPM();
      currentSetpointRPM = targetRPM;

      if (targetRPM == 0.0) {
        launcherController.setReference(0.0, ControlType.kVelocity);
        isAtSpeedLatched = false;
      } else {

        double ff =
            Constants.FuelSystemConstants.LAUNCH_KS * Math.signum(targetRPM)
            + Constants.FuelSystemConstants.LAUNCH_KV * targetRPM;

        launcherController.setReference(
            targetRPM,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ff);

        isAtSpeedLatched = false; // reset latch when new shot requested
      }

      lastState = currentState;
    }

    // --- Stable atSpeed logic (with hysteresis) ---

    double error = currentSetpointRPM - launcherEncoder.getVelocity();

    if (currentSetpointRPM > 0.0) {

      // Tight tolerance to latch true
      if (!isAtSpeedLatched && Math.abs(error) < rpmTolerance) {
        isAtSpeedLatched = true;
      }

      // Wider tolerance to unlatch
      if (isAtSpeedLatched && Math.abs(error) > rpmTolerance * 2.0) {
        isAtSpeedLatched = false;
      }

    } else {
      isAtSpeedLatched = false;
    }
  
    Dashboard.logBoolean("Launcher At Speed", () -> isAtSpeedLatched);
    Dashboard.logNumber("Launcher Target RPM", () -> currentSetpointRPM);
    Dashboard.logNumber("Launcher Actual RPM", () -> launcherEncoder.getVelocity());
    Dashboard.logNumber("Launcher RPM Error", () -> error);
    Dashboard.logString("Launcher State", () -> currentState.toString());
  }
}