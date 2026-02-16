package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Agitator;


/**
 * Command that starts the launch motor at the "far" speed immediately,
 * waits three seconds, then starts the intake and kicker collect commands
 * running in parallel until interrupted.
 *
 * Usage: new LaunchThenCollect(launch, intake, kicker)
 */
public class FarLaunchSequence extends ParallelCommandGroup {
  /**
   * @param launch Launch subsystem (provides LaunchFar())
   * @param intake Intake subsystem (provides IntakeLaunch())
   * @param kicker Kicker subsystem (provides KickerFarLaunch())
   * @param agitator Agitator subsystem (provides AgitatorRun())
   * 
   */
  public FarLaunchSequence(Launcher launch, 
                          Intake intake, 
                          Kicker kicker, 
                          Agitator agitator) {

       addCommands(
        new InstantCommand(() ->
          launch.setState(Launcher.LauncherState.LAUNCH_FAR)
        ),

        new WaitCommand(1.0),

        new InstantCommand(() -> {
          intake.setState(Intake.State.INTAKE_LAUNCH);
          kicker.setState(Kicker.State.KICKER_FAR_LAUNCH);
          agitator.setState(Agitator.State.AGITATOR_LAUNCH);
        })
    );
 }
        
}
