// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Hopper;


public class AutoRotateLaunch extends SequentialCommandGroup {

    public AutoRotateLaunch(Swerve swerve, Launcher launcher, Hopper hopper) {
        double driveSpeed = 0.5; // m/s
        double slowDownDistance = 0.5; // meters, start slowing down near target

        addCommands(
            //Rotate robot to 180 degrees without moving
            new AutoSwerve(swerve, 0, 0, 0.0, 180.0, slowDownDistance, true),

            //Run launcher close for 5 seconds while stationary
            new WaitCommand(0.1), // small delay to ensure rotation completes
            new SequentialCommandGroup(
                new WaitCommand(0.1), // optional short pause
                new StartEndCommand(
                    () -> hopper.setState(Hopper.State.EXTENDING),
                    () -> hopper.setState(Hopper.State.IDLE)
                ).withTimeout(1.0),
                new StartEndCommand(
                    () -> launcher.setState(Launcher.State.LAUNCH_CLOSE),
                    () -> launcher.setState(Launcher.State.IDLE)
                ).withTimeout(5.0))
            );

    }
}