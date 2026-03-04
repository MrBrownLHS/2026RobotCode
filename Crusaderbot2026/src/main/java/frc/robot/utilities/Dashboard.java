// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;


/** Add your docs here. */
public final class Dashboard {
    private Dashboard() {}
    public static void logNumber(String key, DoubleSupplier supplier) {
        Logger.recordOutput(key, supplier.getAsDouble());
    }

    public static void logBoolean(String key, BooleanSupplier supplier) {
        Logger.recordOutput(key, supplier.getAsBoolean());
    }

    public static void logString(String key, Supplier<String> supplier) {
        Logger.recordOutput(key, supplier.get());
    }

    public static void logPose2d(String key, Supplier<Pose2d> supplier) {
        Logger.recordOutput(key, supplier.get());
    }

     public static void logRotation2d(String key, Supplier<Rotation2d> supplier) {
        Logger.recordOutput(key, supplier.get());
    }

    public static void logTranslation2d(String key, Supplier<Translation2d> supplier) {
        Logger.recordOutput(key, supplier.get());
    }
    
    public static void logSwerveModuleState(String key, Supplier<SwerveModuleState> supplier) {
        Logger.recordOutput(key, supplier.get());
    }
    public static void logDouble(String key, DoubleSupplier supplier) {
        Logger.recordOutput(key, supplier.getAsDouble());
    }

    public static void logDoubleArray(String key, double[] values) {
        Logger.recordOutput(key, values);
    }



    

}