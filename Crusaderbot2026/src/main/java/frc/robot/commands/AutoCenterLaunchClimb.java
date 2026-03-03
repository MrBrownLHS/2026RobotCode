// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Climber;


public class AutoCenterLaunchClimb extends SequentialCommandGroup {

    public AutoCenterLaunchClimb(Swerve swerve, Launcher launcher, Climber climber) {
        double driveSpeed = 0.5; // m/s
        double slowDownDistance = 0.5; // meters, start slowing down near target

        addCommands(
            //Rotate robot to 180 degrees without moving
            new AutoSwerve(swerve, 0, 0, 0.0, 180.0, slowDownDistance, true),

            //Run launcher close for 5 seconds while stationary
            new WaitCommand(0.1), // small delay to ensure rotation completes
            new SequentialCommandGroup(
                new WaitCommand(0.1), // optional short pause
                new RunCommand(() -> launcher.setState(Launcher.State.LAUNCH_CLOSE)),
                new WaitCommand(5.0), // spin-up time
                new RunCommand(() -> launcher.setState(Launcher.State.IDLE))
            ),

            //Drive forward ~4 ft (~1.22 meters)
            new AutoSwerve(swerve, 1.22, 0, driveSpeed, 0.0, slowDownDistance, false),

            //Extend the climber
            new RunCommand(() -> climber.setState(Climber.State.EXTENDING)),

            //Drive backward ~6 in (~0.15 meters)
            new AutoSwerve(swerve, 1.22 - 0.15, 0, driveSpeed, 0.0, slowDownDistance, false),

            //Retract the climber
            new RunCommand(() -> climber.setState(Climber.State.CLIMBING))
        );
    }
}