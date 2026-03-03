package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

public class Launcher extends SubsystemBase {

  public enum State {
    IDLE,
    LAUNCH_FAR,
    LAUNCH_CLOSE,
    LAUNCH_COLLECT
  }

  private State currentState = State.IDLE;

  private final SparkMax launcherMotor;
  private final RelativeEncoder launcherEncoder;
  private final SparkClosedLoopController launcherController;

  // Active setpoint tracking (prevents transient errors in atSpeed)
  private double currentSetpointRPM = 0.0;

  // Tunable values (from Constants)
  private final double launchFarRPM = Constants.FuelSystemConstants.LAUNCH_FAR_RPM;
  private final double launchCloseRPM = Constants.FuelSystemConstants.LAUNCH_CLOSE_RPM;
  private final double launchCollectRPM = Constants.FuelSystemConstants.LAUNCH_COLLECT_RPM;
  private final double rpmTolerance = Constants.FuelSystemConstants.LAUNCH_RPM_TOLERANCE;

  private final ShuffleboardTab fuelTab = Shuffleboard.getTab("Fuel System");

  public Launcher() {

    launcherMotor = new SparkMax(
        Constants.FuelSystemConstants.LAUNCH_MOTOR_1_ID,
        MotorType.kBrushless);

    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kCoast);
    config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
    config.secondaryCurrentLimit(Constants.MotorConstants.MAX_CURRENT_LIMIT_NEO);
    config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);

    config.closedLoop
        .pid(
            Constants.FuelSystemConstants.LAUNCH_P,
            Constants.FuelSystemConstants.LAUNCH_I,
            Constants.FuelSystemConstants.LAUNCH_D)
        .outputRange(-1.0, 1.0);

    // APPLY CONFIGURATION (CRITICAL)
    launcherMotor.configure(
        config,
        SparkBase.ResetMode.kResetSafeParameters,
        SparkBase.PersistMode.kPersistParameters);

    launcherEncoder = launcherMotor.getEncoder();
    launcherController = launcherMotor.getClosedLoopController();

    // Explicit velocity units (RPM by default — no conversion needed)

    // Ensure motor starts stopped in velocity mode
    launcherController.setReference(0.0, ControlType.kVelocity, ClosedLoopSlot.kSlot0, 0.0);
    currentSetpointRPM = 0.0;

    // Telemetry (created once — never in periodic)
    fuelTab.addString("Launcher State", () -> currentState.toString());
    fuelTab.addNumber("Launcher Target RPM", () -> currentSetpointRPM);
    fuelTab.addNumber("Launcher Actual RPM", launcherEncoder::getVelocity);
    fuelTab.addNumber("Launcher Error",
        () -> currentSetpointRPM - launcherEncoder.getVelocity());
    fuelTab.addBoolean("Launcher At Speed", this::atSpeed);
    fuelTab.addNumber("Launcher Output %", launcherMotor::getAppliedOutput);
    fuelTab.addNumber("Launcher Bus Voltage", launcherMotor::getBusVoltage);
  }

  public void setState(State state) {
    currentState = state;
  }

  public State getState() {
    return currentState;
  }

  public boolean atSpeed() {
    return Math.abs(currentSetpointRPM - launcherEncoder.getVelocity())
        < rpmTolerance
        && currentSetpointRPM > 0.0;  // Prevent "true at idle"
  }

  private double getTargetRPM() {
    switch (currentState) {
      case LAUNCH_FAR:
        return launchFarRPM;
      case LAUNCH_CLOSE:
        return launchCloseRPM;
      case LAUNCH_COLLECT:
        return launchCollectRPM;
      default:
        return 0.0;
    }
  }

  @Override
  public void periodic() {

    double targetRPM = getTargetRPM();

    if (targetRPM == 0.0) {
      launcherController.setReference(0.0, ControlType.kVelocity);
      currentSetpointRPM = 0.0;
      return;
    }

    // Feedforward in volts
    // IMPORTANT: kV must be volts per RPM
    double ff = Constants.FuelSystemConstants.LAUNCH_KS * Math.signum(targetRPM)
        + Constants.FuelSystemConstants.LAUNCH_KV * targetRPM;

    launcherController.setReference(
        targetRPM,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        ff);

    currentSetpointRPM = targetRPM;
  }
}