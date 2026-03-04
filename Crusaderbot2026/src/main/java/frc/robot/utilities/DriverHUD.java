// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;


public class DriverHUD {

    // Log numeric value
    public static void logNumber(String name, DoubleSupplier value) {
        SmartDashboard.putNumber(name, value.getAsDouble());
    }

    // Log boolean value
    public static void logBoolean(String name, BooleanSupplier value) {
        SmartDashboard.putBoolean(name, value.getAsBoolean());
    }

    // Log string value
    public static void logString(String name, Supplier<String> value) {
        SmartDashboard.putString(name, value.get());
    }

    // Log Pose2d
    public static void logPose2d(String name, Supplier<Pose2d> value) {
        Pose2d pose = value.get();
        SmartDashboard.putNumber(name + " X", pose.getX());
        SmartDashboard.putNumber(name + " Y", pose.getY());
        SmartDashboard.putNumber(name + " Heading", pose.getRotation().getDegrees());
    }

    // Log array of doubles
    public static void logDoubleArray(String name, double[] values) {
        SmartDashboard.putNumberArray(name, values);
    }

    // Convenience logging helpers for common subsystems
    public static void logSwerve(Swerve swerve) {
        // Pose and heading
        logPose2d("Robot Pose", swerve::getPose);
        logNumber("Gyro Heading", swerve::getHeading);
    }

    public static void logLauncher(Launcher launcher) {
        // Launcher exposes atSpeed(); more detailed launcher telemetry is logged inside Launcher.periodic()
        logBoolean("Launcher At Speed", launcher::atSpeed);
    }
    
    public static void logReadyFlags(Launcher launcher, Intake intake) {
        logBoolean("ReadyToShoot", launcher::atSpeed);
        logBoolean("ReadyToCollect", () -> intake.getState() == Intake.State.INTAKE_COLLECT);
    }

    public static void logSuperSystem(SuperSystem superSystem) {
        logString("SuperSystem Wanted State", () -> superSystem.getWantedState().toString());
    }

    public static void logClimber(Climber climber) {
        logString("Climber State", () -> climber.getState().toString());
    }
}
