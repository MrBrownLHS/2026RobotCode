package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;


/**
 * Command that starts the launch motor at the "short" speed immediately,
 * waits three seconds, then starts the intake and index collect commands
 * running in parallel until interrupted.
 *
 * Usage: new LaunchThenCollect(launch, intake, index, diverter)
 */
public class ShortLaunchSequence extends ParallelCommandGroup {
  /**
   * @param launch Launch subsystem (provides LaunchFar())
   * @param intake Intake subsystem (provides IntakeCollect())
   * @param kicker Kicker subsystem (provides KickerLaunch())
   
   */
  public ShortLaunchSequence(Launcher launch, Intake intake, Kicker kicker) {
    // Start the launch motor immediately and concurrently run a sequence
    // that waits 3 seconds and then starts intake+index collect in parallel.
    addCommands(
        // Keep launch running from the start
        launch.LaunchClose(),

        // After a 3s delay, start the intake and index collectors in parallel
        new SequentialCommandGroup(
            new WaitCommand(3.0),
            new ParallelCommandGroup(
                kicker.KickerCloseLaunch(),
                intake.IntakeCollect()
            )
        )
    );

    // Explicitly declare requirements so the scheduler knows which
    // subsystems this command will use. Child commands also declare
    // requirements, but adding them here makes intent clear.
    addRequirements(launch, intake, kicker);
  }
}
