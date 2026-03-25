// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import frc.robot.subsystems.RearIntake;
import edu.wpi.first.wpilibj2.command.RunCommand;

/**
 * Hold-to-run command: while active it sets the SuperSystem to COLLECT, and when
 * the command ends it returns the SuperSystem to IDLE. This makes it suitable for
 * binding with whileTrue(...) so releasing the button stops collection immediately.
 */
public class RearCollectCommand extends RunCommand {
  public RearCollectCommand(RearIntake rearIntake) {
    super(
        () -> rearIntake.setState(RearIntake.State.INTAKE_COLLECT),
        rearIntake);
  }
}