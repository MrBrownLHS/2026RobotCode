// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperSystem extends SubsystemBase {
  /** Creates a new SuperSystem. */
  public enum WantedState {
    IDLE,
    COLLECT,
    LAUNCH_FAR,
    LAUNCH_CLOSE,
    REVERSE
  }

  private WantedState wantedState = WantedState.IDLE;

  private final Launcher launcher;
  private final Kicker kicker;
  private final Intake intake;
  private final Hopper hopper;
  private final Agitator agitator;

  public SuperSystem(Launcher launcher, Kicker kicker, Intake intake, Hopper hopper, Agitator agitator) {
    this.launcher = launcher;
    this.kicker = kicker;
    this.intake = intake;
    this.hopper = hopper;
    this.agitator = agitator;
  }

  public void setWantedState(WantedState newWantedState) {
    wantedState = newWantedState;
  }

  @Override
  public void periodic() {
      switch (wantedState) {
        case IDLE:
          launcher.setState(Launcher.State.IDLE);
          kicker.setState(Kicker.State.IDLE);
          intake.setState(Intake.State.IDLE);
          hopper.setState(Hopper.State.IDLE);
          agitator.setState(Agitator.State.IDLE);
          break;

        case COLLECT:
          launcher.setState(Launcher.State.LAUNCH_COLLECT);
          kicker.setState(Kicker.State.KICK_COLLECT);
          intake.setState(Intake.State.INTAKE_COLLECT);
          hopper.setState(Hopper.State.EXTENDING);
          agitator.setState(Agitator.State.IDLE);
          break;

        case LAUNCH_FAR:
          launcher.setState(Launcher.State.LAUNCH_FAR);

          if (launcher.atSpeed()) {
            kicker.setState(Kicker.State.KICK_FAR);
            intake.setState(Intake.State.INTAKE_LAUNCH);
            hopper.setState(Hopper.State.RETRACTING);
            agitator.setState(Agitator.State.AGITATE);
          } else {
            kicker.setState(Kicker.State.IDLE);
            intake.setState(Intake.State.IDLE);
            hopper.setState(Hopper.State.IDLE);
            agitator.setState(Agitator.State.IDLE);
          }
          break;

        case LAUNCH_CLOSE:
          launcher.setState(Launcher.State.LAUNCH_CLOSE);

          if (launcher.atSpeed()) {
            kicker.setState(Kicker.State.KICK_CLOSE);
            intake.setState(Intake.State.INTAKE_LAUNCH);
            hopper.setState(Hopper.State.RETRACTING);
            agitator.setState(Agitator.State.AGITATE);
          } else {
            kicker.setState(Kicker.State.IDLE);
            intake.setState(Intake.State.IDLE);
            hopper.setState(Hopper.State.IDLE);
            agitator.setState(Agitator.State.IDLE);
          }
          break;

        case REVERSE:
          kicker.setState(Kicker.State.KICK_REVERSE);
          intake.setState(Intake.State.INTAKE_REVERSE);
          agitator.setState(Agitator.State.REVERSE);
          break;
      }
  }
}
