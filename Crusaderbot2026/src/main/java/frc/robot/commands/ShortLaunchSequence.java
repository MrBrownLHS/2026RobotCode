package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.subsystems.Launch;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Index;
import frc.robot.subsystems.LaunchDiverter;

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
   * @param index Index subsystem (provides IndexCollect())
   * @param diverter LaunchDiverter subsystem (provides LaunchDivertertoLaunch())
   */
  public ShortLaunchSequence(Launch launch, Intake intake, Index index, LaunchDiverter diverter) {
    // Start the launch motor immediately and concurrently run a sequence
    // that waits 3 seconds and then starts intake+index collect in parallel.
    addCommands(
        // Keep launch running from the start
        launch.LaunchClose(),

        // After a 3s delay, start the intake and index collectors in parallel
        new SequentialCommandGroup(
            new WaitCommand(3.0),
            new ParallelCommandGroup(
                diverter.LaunchDivertertoLaunch(),
                index.IndexCollect(),
                intake.IntakeCollect()
            )
        )
    );

    // Explicitly declare requirements so the scheduler knows which
    // subsystems this command will use. Child commands also declare
    // requirements, but adding them here makes intent clear.
    addRequirements(launch, intake, index, diverter);
  }
}
