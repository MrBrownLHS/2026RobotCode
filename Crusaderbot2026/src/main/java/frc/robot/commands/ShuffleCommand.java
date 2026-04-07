package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.RearIntake;
import frc.robot.subsystems.RearIntakeLift;

public class ShuffleCommand extends Command {

  private enum Phase {
    EXTENDING,
    PAUSE_AT_EXTENDED,
    SHUFFLE_IN,
    PAUSE_AT_SHUFFLE_IN
  }

  private final Hopper hopper;
  private final RearIntake rearIntake;
  private final RearIntakeLift rearIntakeLift;
  private final Timer timer = new Timer();

  private Phase currentPhase = Phase.EXTENDING;

  private static final double PAUSE_SECONDS = 0.3;

  public ShuffleCommand(Hopper hopper, RearIntake rearIntake, RearIntakeLift rearIntakeLift) {
    this.hopper = hopper;
    this.rearIntake = rearIntake;
    this.rearIntakeLift = rearIntakeLift;

    addRequirements(hopper, rearIntake, rearIntakeLift);
  }

  @Override
  public void initialize() {
    currentPhase = Phase.EXTENDING;
    timer.stop();
    timer.reset();
  }

  @Override
  public void execute() {
    // Keep rear intake reversing throughout shuffle
    rearIntake.setState(RearIntake.State.INTAKE_REVERSE);

    switch (currentPhase) {
      case EXTENDING:
        hopper.setState(Hopper.State.EXTENDING);
        rearIntakeLift.setState(RearIntakeLift.State.LIFTED);

        if (hopper.isExtended()) {
          hopper.setState(Hopper.State.IDLE);
          timer.restart();
          currentPhase = Phase.PAUSE_AT_EXTENDED;
        }
        break;

      case PAUSE_AT_EXTENDED:
        hopper.setState(Hopper.State.IDLE);
        rearIntakeLift.setState(RearIntakeLift.State.LIFTED);

        if (timer.hasElapsed(PAUSE_SECONDS)) {
          timer.stop();
          timer.reset();
          currentPhase = Phase.SHUFFLE_IN;
        }
        break;

      case SHUFFLE_IN:
        hopper.setState(Hopper.State.SHUFFLE_IN);
        rearIntakeLift.setState(RearIntakeLift.State.STORED);

        if (hopper.atShuffleIn()) {
          hopper.setState(Hopper.State.IDLE);
          timer.restart();
          currentPhase = Phase.PAUSE_AT_SHUFFLE_IN;
        }
        break;

      case PAUSE_AT_SHUFFLE_IN:
        hopper.setState(Hopper.State.IDLE);
        rearIntakeLift.setState(RearIntakeLift.State.STORED);

        if (timer.hasElapsed(PAUSE_SECONDS)) {
          timer.stop();
          timer.reset();
          currentPhase = Phase.EXTENDING;
        }
        break;
    }
  }

  @Override
  public void end(boolean interrupted) {
    hopper.setState(Hopper.State.IDLE);
    rearIntake.setState(RearIntake.State.IDLE);
    rearIntakeLift.setState(RearIntakeLift.State.IDLE);
    timer.stop();
    timer.reset();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}