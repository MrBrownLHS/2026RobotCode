// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.SuperSystem;
import frc.robot.subsystems.Launcher;

public class LaunchCloseCommand extends SequentialCommandGroup {

  public LaunchCloseCommand(SuperSystem superSystem, Launcher launcher) {

    addCommands(

      new InstantCommand(
          () -> superSystem.setWantedState(SuperSystem.WantedState.LAUNCH_CLOSE),
          superSystem),

      new WaitUntilCommand(launcher::atSpeed)
    );
  }
}