package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Constants;
import frc.robot.utilities.Dashboard;

public class Hopper extends SubsystemBase {

    public enum State {
        IDLE,
        EXTENDING,
        RETRACTING,
        SHUFFLE_IN
    }

    private State currentState = State.IDLE;

    private final SparkMax m_Hopper;
    private final RelativeEncoder hopperEncoder;

    // Tunable Positions
    private final double openPosition = Constants.FuelSystemConstants.HOPPER_EXTEND_POSITION;
    private final double shufflePosition = Constants.FuelSystemConstants.HOPPER_SHUFFLE_POSITION;
    private final double retractedPosition = Constants.FuelSystemConstants.HOPPER_RETRACT_POSITION;

    // Tunable control
    private final double positionTolerance = 0.5;
    private final double extendSpeed = Constants.FuelSystemConstants.HOPPER_EXTEND_SPEED;
    private final double retractSpeed = Constants.FuelSystemConstants.HOPPER_RETRACT_SPEED;

    private double currentTarget = 0.0;

    public Hopper() {

        m_Hopper = new SparkMax(
            Constants.FuelSystemConstants.HOPPER_MOTOR_ID,
            MotorType.kBrushless
        );

        SparkMaxConfig config = new SparkMaxConfig();

        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
        config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);

        m_Hopper.configure(
            config,
            SparkBase.ResetMode.kResetSafeParameters,
            SparkBase.PersistMode.kPersistParameters
        );

        hopperEncoder = m_Hopper.getEncoder();

        // Assume starting at home
        hopperEncoder.setPosition(0.0);
        m_Hopper.set(0.0);
    }

    /* =============================
       Public State Control
       ============================= */

    public void setState(State newState) {
        currentState = newState;
    }

    public State getState() {
        return currentState;
    }

    public void stop() {
        currentState = State.IDLE;
        m_Hopper.set(0.0);
    }

    /* =============================
       Encoder Utilities
       ============================= */

    public double getPosition() {
        return hopperEncoder.getPosition();
    }

    public void zeroEncoder() {
        hopperEncoder.setPosition(0.0);
    }

    public boolean atTarget() {
        return Math.abs(getPosition() - currentTarget) < positionTolerance;
    }

    public boolean isExtended() {
        return Math.abs(getPosition() - openPosition) < positionTolerance;
    }

    public boolean isRetracted() {
        return Math.abs(getPosition() - retractedPosition) < positionTolerance;
    }

    public boolean atShuffleIn() {
        return Math.abs(getPosition() - shufflePosition) < positionTolerance;
    }

    /* =============================
       Internal Helpers
       ============================= */

    private double moveTowardTarget(
        double target,
        double positiveSpeed,
        double negativeSpeed
    ) {
        double position = getPosition();
        double error = target - position;

        if (Math.abs(error) < positionTolerance) {
            return 0.0;
        }

        if (error > 0) {
            return positiveSpeed;
        } else {
            return negativeSpeed;
        }
    }

    /* =============================
       Periodic
       ============================= */

    @Override
    public void periodic() {

        double output = 0.0;

        switch (currentState) {

            case IDLE:
                currentTarget = getPosition();
                output = 0.0;
                break;

            case EXTENDING:
                currentTarget = openPosition;
                output = moveTowardTarget(
                    currentTarget,
                    extendSpeed,
                    retractSpeed
                );
                break;

            case RETRACTING:
                currentTarget = retractedPosition;
                output = moveTowardTarget(
                    currentTarget,
                    extendSpeed,
                    retractSpeed
                );
                break;

            case SHUFFLE_IN:
                currentTarget = shufflePosition;
                output = moveTowardTarget(
                    currentTarget,
                    extendSpeed,
                    retractSpeed
                );
                break;
        }

        m_Hopper.set(output);

        /* =============================
           Dashboard Logging
           ============================= */

        Dashboard.logString("Hopper State", () -> currentState.toString());
        Dashboard.logNumber("Hopper Position", this::getPosition);
        Dashboard.logNumber("Hopper Target", () -> currentTarget);
        Dashboard.logBoolean("Hopper At Target", this::atTarget);
        Dashboard.logBoolean("Hopper Extended", this::isExtended);
        Dashboard.logBoolean("Hopper Retracted", this::isRetracted);
        Dashboard.logBoolean("Hopper At Shuffle In", this::atShuffleIn);
    }
}