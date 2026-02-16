package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Agitator;


/**
 * Command that starts the launch motor at the "short" speed immediately,
 * waits one second, then starts the intake and index collect commands
 * running in parallel until interrupted.
 *
 * Usage: new LaunchThenCollect(launch, intake, index, diverter)
 */
public class ShortLaunchSequence extends ParallelCommandGroup {
  /**
   * @param launch Launch subsystem (provides LaunchFar())
   * @param intake Intake subsystem (provides IntakeCollect())
   * @param kicker Kicker subsystem (provides KickerCloseLaunch())
   * @param agitator Agitator subsystem (provides AgitatorRun())
   * 
   
   */
  public ShortLaunchSequence(Launcher launch, Intake intake, Kicker kicker, Agitator agitator) {
    // Start the launch motor immediately and concurrently run a sequence
    // that waits 3 seconds and then starts intake+index collect in parallel.
    addCommands(
        // Keep launch running from the start
        launch.runClose().alongWith(new WaitCommand(1.0)),
        // After a 1s delay, start the intake and index collectors in parallel
        new ParallelCommandGroup(
            kicker.KickerCloseLaunch(),
            intake.IntakeLaunch(),
            agitator.AgitatorRun()  
        )
    );
  }
}
