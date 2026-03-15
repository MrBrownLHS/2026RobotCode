// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SuperSystem;


public class AutoRotateLaunch extends SequentialCommandGroup {

    public AutoRotateLaunch(Swerve swerve, SuperSystem superSystem) {

        addCommands(

            // Reset gyro heading
            new RunCommand(() -> swerve.resetHeading(), swerve).withTimeout(0.05),

            // Back up ~2 ft
            new RunCommand(
                () -> swerve.drive(new Translation2d(-0.6, 0), 0, false, false),
                swerve
            ).withTimeout(1.2),

            new WaitCommand(0.2),

            // Turn to 180 degrees
            new RunCommand(
                () -> swerve.turnToAngle(180),
                swerve
            )
            .until(() -> swerve.atAngle(180))
            .withTimeout(3.0),

            new WaitCommand(0.2),

            // Activate the launcher system
            new RunCommand(
                () -> superSystem.setWantedState(SuperSystem.WantedState.LAUNCH_CLOSE),
                superSystem
            ).withTimeout(5.0),

            // Return to idle
            new RunCommand(
                () -> superSystem.setWantedState(SuperSystem.WantedState.IDLE),
                superSystem
            ).withTimeout(0.05)
        );
    }
}