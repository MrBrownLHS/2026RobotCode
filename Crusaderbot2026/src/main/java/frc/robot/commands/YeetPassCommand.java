// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SuperSystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class YeetPassCommand extends StartEndCommand {

  public YeetPassCommand(SuperSystem superSystem) {
    super(
        () -> superSystem.setFrontWantedState(SuperSystem.FrontWantedState.YEET_PASS),
        // onEnd: return to IDLE when the command ends (button released)
        () -> superSystem.setFrontWantedState(SuperSystem.FrontWantedState.IDLE)
    );
  }
}

