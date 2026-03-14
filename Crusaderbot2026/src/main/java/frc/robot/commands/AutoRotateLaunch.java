// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Hopper;


public class AutoRotateLaunch extends SequentialCommandGroup {

    public AutoRotateLaunch(Swerve swerve, Launcher launcher, Hopper hopper) {
       
        addCommands(

            // Reset heading so 180° is predictable
            new RunCommand(() -> swerve.resetHeading(), swerve).withTimeout(0.05),

            // Step 1: Back up about 2 feet
            new RunCommand(
                () -> swerve.drive(new Translation2d(-0.6, 0), 0, false, false),
                swerve
            ).withTimeout(1.2),

            new WaitCommand(0.2),

            // Step 2: Rotate to 180° using gyro
            new RunCommand(
                () -> {
                    double error = MathUtil.inputModulus(180 - swerve.getHeading(), -180, 180);

                    // Proportional rotation control
                    double rotSpeed = error * 0.01;

                    // Limit rotation speed to prevent spinning wildly
                    rotSpeed = MathUtil.clamp(rotSpeed, -0.8, 0.8);

                    swerve.drive(new Translation2d(0, 0), rotSpeed, false, false);
                },
                swerve
            )
            .until(() -> Math.abs(MathUtil.inputModulus(180 - swerve.getHeading(), -180, 180)) < 5)
            .withTimeout(3.0),

            new WaitCommand(0.2),

            // Step 3: Spin launcher and feed hopper
            new ParallelCommandGroup(

                // Launcher spins up and stays running
                new RunCommand(
                    () -> launcher.setState(Launcher.State.LAUNCH_CLOSE),
                    launcher
                ).withTimeout(5.0),

                // Hopper feeds after a short delay
                new SequentialCommandGroup(
                    new WaitCommand(1.0),
                    new RunCommand(
                        () -> hopper.setState(Hopper.State.EXTENDING),
                        hopper
                    ).withTimeout(1.5)
                )
            ),

            // Final stop of all mechanisms
            new RunCommand(() -> {
                launcher.setState(Launcher.State.IDLE);
                hopper.setState(Hopper.State.IDLE);
                swerve.stop();
            }).withTimeout(0.05)
        );
    }
}