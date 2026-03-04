// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.Launcher;

public class LaunchFarCommand extends SequentialCommandGroup {

  public LaunchFarCommand(SuperSystem superSystem, Launcher launcher) {

    addCommands(

      // Set system to launch mode (spin up only)
      new InstantCommand(
          () -> superSystem.setWantedState(SuperSystem.WantedState.LAUNCH_FAR),
          superSystem),

      // Wait until flywheel reaches speed
      new WaitUntilCommand(launcher::atSpeed)

      // Feeding continues because SuperSystem is in LAUNCH_FAR
    );
  }
}

