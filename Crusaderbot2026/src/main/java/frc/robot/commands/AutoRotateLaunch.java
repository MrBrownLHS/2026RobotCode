package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.RearIntake;
import frc.robot.subsystems.RearIntakeLift;

public class AutoRotateLaunch extends SequentialCommandGroup {

    public AutoRotateLaunch(
        Swerve swerve, 
        SuperSystem superSystem, 
        Hopper hopper, 
        RearIntake rearIntake, 
        RearIntakeLift rearIntakeLift) {

        addCommands(

             // Back up
            new RunCommand(
                () -> swerve.drive(new Translation2d(-0.6, 0), 0.0, false, false),
                swerve
            ).withTimeout(1.2),

            // Stop drive
            new InstantCommand(
                () -> swerve.drive(new Translation2d(0.0, 0.0), 0.0, false, false),
                swerve
            ),

            new WaitCommand(0.2),

            // Timed rotate instead of turnToAngle(180)
            new RunCommand(
                () -> swerve.drive(new Translation2d(0.0, 0.0), 0.5, false, false),
                swerve
            ).withTimeout(3.5),

            // Stop drive after rotation
            new InstantCommand(
                () -> swerve.drive(new Translation2d(0.0, 0.0), 0.0, false, false),
                swerve
            ),

            new WaitCommand(0.2),

            // Extend hopper
            new InstantCommand(
                () -> superSystem.setRearWantedState(SuperSystem.RearWantedState.EXTEND_HOPPER),
                superSystem
            ),

            new WaitCommand(0.5),

            // Run launch + shuffle at the same time for 5 seconds
            new ParallelDeadlineGroup(
                new WaitCommand(5.0),

                new RunCommand(
                    () -> superSystem.setFrontWantedState(SuperSystem.FrontWantedState.LAUNCH_CLOSE),
                    superSystem
                ),

                new ShuffleCommand(hopper, rearIntake, rearIntakeLift)
            ),

            // Stop everything cleanly
            new InstantCommand(() -> {
                superSystem.setFrontWantedState(SuperSystem.FrontWantedState.IDLE);
                superSystem.setRearWantedState(SuperSystem.RearWantedState.IDLE);
                superSystem.setRearLiftWantedState(SuperSystem.RearLiftWantedState.IDLE);
                swerve.drive(new Translation2d(0.0, 0.0), 0.0, false, false);
            }, superSystem, swerve)
        );
    }
}