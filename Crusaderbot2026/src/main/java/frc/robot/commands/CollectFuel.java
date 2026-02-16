// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Kicker;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;

/**
 * Parallel command group that runs the intake, index, launch, divert collect
 * commands simultaneously. Construct this with the subsystem instances so
 * the command group can use their RunCommand factory methods.
 */
public class CollectFuel extends ParallelCommandGroup {
  /**
   * Create a new CollectFuel command group.
   *
   * @param kicker the Kicker subsystem (provides KickerCollect())
   * @param intake the Intake subsystem (provides IntakeCollect())
   * @param launch the Launch subsystem (provides LaunchCollect())
 
   */
  public CollectFuel(Kicker kicker, Intake intake, Launcher launch){
    // Add the RunCommand instances returned by each subsystem's "Collect"
    // factory method. These RunCommands already declare the subsystem as a
    // requirement, so the ParallelCommandGroup knows which subsystems are in
    // use.
  
    addCommands(
        kicker.KickerCollect(),
        intake.IntakeCollect(),
        launch.runCollect()        
    );

    // Optionally declare requirements explicitly (not strictly necessary
    // because the child commands already require these subsystems).
    addRequirements(kicker, intake, launch);
  }
}
