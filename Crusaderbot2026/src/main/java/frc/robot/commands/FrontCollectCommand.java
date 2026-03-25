// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.SuperSystem;

/**
 * Hold-to-run command: while active it sets the SuperSystem to COLLECT, and when
 * the command ends it returns the SuperSystem to IDLE. This makes it suitable for
 * binding with whileTrue(...) so releasing the button stops collection immediately.
 */
public class FrontCollectCommand extends StartEndCommand {
  public FrontCollectCommand(SuperSystem superSystem) {
    super(
        () -> superSystem.setFrontWantedState(SuperSystem.FrontWantedState.FRONT_COLLECT),
        () -> superSystem.setFrontWantedState(SuperSystem.FrontWantedState.IDLE),
        superSystem);
  }
}