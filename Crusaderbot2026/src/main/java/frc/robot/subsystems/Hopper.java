package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Timer;
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

    private final SparkMax m_Hopper;
    private final RelativeEncoder hopperEncoder;

    // Tunable Positions
    private final double openPosition = Constants.FuelSystemConstants.HOPPER_EXTEND_POSITION;
    private final double shufflePosition = Constants.FuelSystemConstants.HOPPER_SHUFFLE_POSITION;
    private final double retractedPosition = Constants.FuelSystemConstants.HOPPER_RETRACT_POSITION;

    // Tunable control
    private final double positionTolerance = 0.5;
    private final double slowZone = 2.0; // begin slowing this far from target

    // Normal extend/retract speeds
    private final double fastExtendSpeed = 0.18;
    private final double slowExtendSpeed = 0.10;
    private final double fastRetractSpeed = -0.18;
    private final double slowRetractSpeed = -0.10;

    // Shuffle speeds (a little gentler)
    private final double fastShuffleExtendSpeed = 0.12;
    private final double slowShuffleExtendSpeed = 0.08;
    private final double fastShuffleRetractSpeed = -0.12;
    private final double slowShuffleRetractSpeed = -0.08;

    // Shuffle timing
    private final double shufflePauseSeconds = 0.10;
    private final Timer shufflePauseTimer = new Timer();
    private boolean shufflePaused = false;

    // Shuffle control
    private boolean shuffleGoingOut = true;
    private double currentTarget = openPosition;

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
        if (newState != currentState) {
            currentState = newState;

            if (newState == State.SHUFFLE) {
                shuffleGoingOut = true;
                currentTarget = openPosition;
                shufflePaused = false;
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

    /* =============================
       Internal Helpers
       ============================= */

    private double moveTowardTarget(
        double target,
        double fastPositiveSpeed,
        double slowPositiveSpeed,
        double fastNegativeSpeed,
        double slowNegativeSpeed
    ) {
        double position = getPosition();
        double error = target - position;
        double absError = Math.abs(error);

        if (absError < positionTolerance) {
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
       Periodic
       ============================= */

    @Override
    public void periodic() {

        double output = 0.0;
        double position = getPosition();

        switch (currentState) {

            case IDLE:
                output = 0.0;
                break;

            case EXTENDING:
                currentTarget = openPosition;
                output = moveTowardTarget(
                    currentTarget,
                    fastExtendSpeed,
                    slowExtendSpeed,
                    fastRetractSpeed,
                    slowRetractSpeed
                );
                break;

            case RETRACTING:
                currentTarget = retractedPosition;
                output = moveTowardTarget(
                    currentTarget,
                    fastExtendSpeed,
                    slowExtendSpeed,
                    fastRetractSpeed,
                    slowRetractSpeed
                );
                break;

            case SHUFFLE:
                if (shufflePaused) {
                    output = 0.0;

                    if (shufflePauseTimer.hasElapsed(shufflePauseSeconds)) {
                        shufflePaused = false;
                        shufflePauseTimer.stop();
                        shufflePauseTimer.reset();

                        shuffleGoingOut = !shuffleGoingOut;
                        currentTarget = shuffleGoingOut ? openPosition : shufflePosition;
                    }
                    break;
                }

                if (Math.abs(position - currentTarget) < positionTolerance) {
                    output = 0.0;
                    shufflePaused = true;
                    shufflePauseTimer.restart();
                    break;
                }

                output = moveTowardTarget(
                    currentTarget,
                    fastShuffleExtendSpeed,
                    slowShuffleExtendSpeed,
                    fastShuffleRetractSpeed,
                    slowShuffleRetractSpeed
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
        Dashboard.logBoolean("Hopper Shuffle Paused", () -> shufflePaused);
    }
}