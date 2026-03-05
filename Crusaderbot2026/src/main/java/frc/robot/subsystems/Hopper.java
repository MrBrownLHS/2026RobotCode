package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Constants;
import frc.robot.utilities.Dashboard;

public class Hopper extends SubsystemBase {

    public enum State {
        IDLE,
        EXTENDING,
        RETRACTING,
        SHUFFLE
    }

    private State currentState = State.IDLE;

    private final SparkMax hopperMotor;
    private final RelativeEncoder hopperEncoder;
    private final SparkClosedLoopController hopperController;

    private final DigitalInput homeSwitch;

    // Tunable Positions
    private final double openPosition = Constants.FuelSystemConstants.HOPPER_EXTEND_POSITION; // ~10

    private final double shufflePosition = Constants.FuelSystemConstants.HOPPER_SHUFFLE_POSITION; // ~5

    private final double positionTolerance = 0.15;

    // Shuffle control
    private boolean shuffleGoingOut = true;
    private double currentTarget = openPosition;

    public Hopper() {

        hopperMotor = new SparkMax(
            Constants.FuelSystemConstants.HOPPER_MOTOR_ID,
            MotorType.kBrushless
        );

        SparkMaxConfig config = new SparkMaxConfig();

        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
        config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);

        // ---- Closed Loop Position Control ----
        config.closedLoop
            .pid(
                0.4,   // kP (start here, tune lightly)
                0.0,   // kI
                0.0)   // kD
            .outputRange(-0.15, 0.15);  // limit speed

        hopperMotor.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        hopperEncoder = hopperMotor.getEncoder();
        hopperController = hopperMotor.getClosedLoopController();

        hopperEncoder.setPosition(0);

        homeSwitch = new DigitalInput(
            Constants.FuelSystemConstants.HOPPER_HOME_SWITCH_DIO
        );
    }

    /* =============================
       Public State Control
       ============================= */

    public void setState(State newState) {
        if (newState != currentState) {
            currentState = newState;

            // Reset shuffle direction when entering shuffle
            if (newState == State.SHUFFLE) {
                shuffleGoingOut = true;
                currentTarget = openPosition;
            }
        }
    }

    public State getState() {
        return currentState;
    }

    public void stop() {
        currentState = State.IDLE;
        hopperMotor.set(0);
    }

    /* =============================
       Home Switch
       ============================= */

    public boolean isHomePressed() {
        return !homeSwitch.get(); // invert if wired NC
    }

    /* =============================
       Periodic
       ============================= */

    @Override
    public void periodic() {

        double position = hopperEncoder.getPosition();

        // Auto re-zero when homed
        if (isHomePressed()) {
            hopperEncoder.setPosition(0);
        }

        switch (currentState) {

            case IDLE:
                hopperMotor.set(0);
                break;

            case EXTENDING:
                hopperController.setReference(
                    openPosition,
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot0);
                break;

            case RETRACTING:
                hopperController.setReference(
                    0,
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot0);
                break;

            case SHUFFLE:

                // If at target, switch direction
                if (Math.abs(position - currentTarget) < positionTolerance) {
                    shuffleGoingOut = !shuffleGoingOut;
                    currentTarget = shuffleGoingOut
                        ? openPosition
                        : shufflePosition;
                }

                hopperController.setReference(
                    currentTarget,
                    ControlType.kPosition,
                    ClosedLoopSlot.kSlot0);

                break;
        }

        Dashboard.logString("Hopper State", () -> currentState.toString());
        Dashboard.logNumber("Hopper Position", hopperEncoder::getPosition);
        Dashboard.logNumber("Hopper Target", () -> currentTarget);
        Dashboard.logBoolean("Home Switch", this::isHomePressed);
    }
}