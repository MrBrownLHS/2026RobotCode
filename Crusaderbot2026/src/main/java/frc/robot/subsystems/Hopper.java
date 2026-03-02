// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.utilities.Constants;

public class Hopper extends SubsystemBase {

    public enum State {
        IDLE,
        EXTENDING,
        RETRACTING
    }

    private State currentState = State.IDLE;

    private final SparkMax hopperMotor;
    private final RelativeEncoder hopperEncoder;

    // Single home (closed) limit switch
    private final DigitalInput homeSwitch;

    // Tunable values (default fallbacks)
    private double extendSpeed = Constants.FuelSystemConstants.HOPPER_EXTEND_SPEED; //Tune during testing
    private double retractSpeed = Constants.FuelSystemConstants.HOPPER_RETRACT_SPEED; //Tune during testing
    private double openPosition = Constants.FuelSystemConstants.HOPPER_OPEN_POSITION; // rotations – tune this

    public Hopper() {

        hopperMotor = new SparkMax(
            Constants.FuelSystemConstants.HOPPER_MOTOR_ID,
            MotorType.kBrushless
        );

        SparkMaxConfig config = new SparkMaxConfig();
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(Constants.MotorConstants.CURRENT_LIMIT_NEO);
        config.voltageCompensation(Constants.MotorConstants.VOLTAGE_COMPENSATION);

        hopperEncoder = hopperMotor.getEncoder();
        hopperEncoder.setPosition(0);

        homeSwitch = new DigitalInput(
            Constants.FuelSystemConstants.HOPPER_HOME_SWITCH_DIO
        );

        // SmartDashboard defaults
        SmartDashboard.putNumber("Hopper Extend Speed", extendSpeed);
        SmartDashboard.putNumber("Hopper Retract Speed", retractSpeed);
        SmartDashboard.putNumber("Hopper Open Position", openPosition);
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
        hopperMotor.set(0);
    }

    /* =============================
       Home Switch
       ============================= */

    public boolean isHomePressed() {
        // Invert if wired normally closed
        return !homeSwitch.get();
    }

    /* =============================
       Periodic
       ============================= */

    @Override
    public void periodic() {

        // Pull live tuning values
        extendSpeed = SmartDashboard.getNumber("Hopper Extend Speed", extendSpeed);
        retractSpeed = SmartDashboard.getNumber("Hopper Retract Speed", retractSpeed);
        openPosition = SmartDashboard.getNumber("Hopper Open Position", openPosition);

        double position = hopperEncoder.getPosition();

        // Auto re-zero if home switch is pressed
        if (isHomePressed()) {
            hopperEncoder.setPosition(0);
        }

        switch (currentState) {

            case IDLE:
                hopperMotor.set(0);
                break;

            case EXTENDING:
                if (position >= openPosition) {
                    hopperMotor.set(0);
                    currentState = State.IDLE;
                } else {
                    hopperMotor.set(extendSpeed);
                }
                break;

            case RETRACTING:
                if (position <= 0 || isHomePressed()) {
                    hopperMotor.set(0);
                    currentState = State.IDLE;
                } else {
                    hopperMotor.set(retractSpeed);
                }
                break;
        }

        /* =============================
           Telemetry
           ============================= */

        SmartDashboard.putString("Hopper State", currentState.toString());
        SmartDashboard.putNumber("Hopper Position", position);
        SmartDashboard.putBoolean("Hopper Home Switch", isHomePressed());
    }
}