// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SuperSystem;
import edu.wpi.first.wpilibj2.command.StartEndCommand;


public class ReverseCollectorCommand extends StartEndCommand {

  public ReverseCollectorCommand(SuperSystem superSystem) {
    super(
        // onStart: set desired state to REVERSE
        () -> superSystem.setWantedState(SuperSystem.WantedState.REVERSE),
        // onEnd: return to IDLE when the command ends (button released)
        () -> superSystem.setWantedState(SuperSystem.WantedState.IDLE),
        superSystem);
  }
}
