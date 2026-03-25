package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Dashboard;

public class SuperSystem extends SubsystemBase {

  /* =============================
     Front Path Wanted States
     Controls:
       - Launcher
       - FrontIntake
       - Agitator
     ============================= */
  public enum FrontWantedState {
    IDLE,
    FRONT_COLLECT,
    LAUNCH_FAR,
    LAUNCH_CLOSE,
    YEET_PASS,
    REVERSE
  }

  /* =============================
     Rear Path Wanted States
     Controls:
       - Hopper
       - RearIntake
     May also temporarily command RearIntakeLift
     when hopper safety/coordination requires it.
     ============================= */
  public enum RearWantedState {
    IDLE,
    EXTEND_HOPPER,
    RETRACT_HOPPER,
    REAR_COLLECT,
    SHUFFLE_HOPPER,
    REVERSE
  }

  /* =============================
     Rear Lift Wanted States
     Independent lift control unless overridden
     by RearWantedState sequencing.
     ============================= */
  public enum RearLiftWantedState {
    IDLE,
    STORED,
    LIFTED,
    EXTENDED,
    SHUFFLE
  }

  private FrontWantedState frontWantedState = FrontWantedState.IDLE;
  private RearWantedState rearWantedState = RearWantedState.IDLE;
  private RearLiftWantedState rearLiftWantedState = RearLiftWantedState.IDLE;

  private final Launcher launcher;
  private final RearIntake rearIntake;
  private final FrontIntake intake;
  private final Hopper hopper;
  private final Agitator agitator;
  private final RearIntakeLift rearIntakeLift;

  public SuperSystem(
      Launcher launcher,
      RearIntake rearIntake,
      FrontIntake intake,
      Hopper hopper,
      Agitator agitator,
      RearIntakeLift rearIntakeLift) {
    this.launcher = launcher;
    this.rearIntake = rearIntake;
    this.intake = intake;
    this.hopper = hopper;
    this.agitator = agitator;
    this.rearIntakeLift = rearIntakeLift;
  }

  /* =============================
     Setters / Getters
     ============================= */

  public void setFrontWantedState(FrontWantedState newWantedState) {
    frontWantedState = newWantedState;
  }

  public FrontWantedState getFrontWantedState() {
    return frontWantedState;
  }

  public void setRearWantedState(RearWantedState newWantedState) {
    rearWantedState = newWantedState;
  }

  public RearWantedState getRearWantedState() {
    return rearWantedState;
  }

  public void setRearLiftWantedState(RearLiftWantedState newWantedState) {
    rearLiftWantedState = newWantedState;
  }

  public RearLiftWantedState getRearLiftWantedState() {
    return rearLiftWantedState;
  }

  public boolean isLauncheratSpeed() {
    return launcher.atSpeed();
  }

  @Override
  public void periodic() {

    /* =============================
       FRONT PATH
       Launcher + Front Intake + Agitator
       ============================= */
    switch (frontWantedState) {

      case IDLE:
        launcher.setState(Launcher.State.IDLE);
        intake.setState(FrontIntake.State.IDLE);
        agitator.setState(Agitator.State.IDLE);
        break;

      case FRONT_COLLECT:
        launcher.setState(Launcher.State.LAUNCH_COLLECT);

        if (launcher.atSpeed()) {
          intake.setState(FrontIntake.State.INTAKE_COLLECT);
          agitator.setState(Agitator.State.IDLE);
        } else {
          intake.setState(FrontIntake.State.IDLE);
          agitator.setState(Agitator.State.IDLE);
        }
        break;

      case LAUNCH_FAR:
        launcher.setState(Launcher.State.LAUNCH_FAR);

        if (launcher.atSpeed()) {
          intake.setState(FrontIntake.State.INTAKE_LAUNCH);
          agitator.setState(Agitator.State.AGITATE);
        } else {
          intake.setState(FrontIntake.State.IDLE);
          agitator.setState(Agitator.State.IDLE);
        }
        break;

      case LAUNCH_CLOSE:
        launcher.setState(Launcher.State.LAUNCH_CLOSE);

        if (launcher.atSpeed()) {
          intake.setState(FrontIntake.State.INTAKE_LAUNCH);
          agitator.setState(Agitator.State.AGITATE);
        } else {
          intake.setState(FrontIntake.State.IDLE);
          agitator.setState(Agitator.State.IDLE);
        }
        break;

      case YEET_PASS:
        launcher.setState(Launcher.State.YEET_PASS);

        if (launcher.atSpeed()) {
          intake.setState(FrontIntake.State.INTAKE_LAUNCH);
          agitator.setState(Agitator.State.AGITATE);
        } else {
          intake.setState(FrontIntake.State.IDLE);
          agitator.setState(Agitator.State.IDLE);
        }
        break;

      case REVERSE:
        launcher.setState(Launcher.State.IDLE);
        intake.setState(FrontIntake.State.INTAKE_REVERSE);
        agitator.setState(Agitator.State.REVERSE);
        break;
    }

    /* =============================
       REAR PATH + LIFT COORDINATION
       Rear path may override lift for safe sequencing.
       If it does not, lift follows rearLiftWantedState.
       ============================= */

    RearIntakeLift.State commandedLiftState = null;

    switch (rearWantedState) {

      case IDLE:
        hopper.setState(Hopper.State.IDLE);
        rearIntake.setState(RearIntake.State.IDLE);
        break;

      case EXTEND_HOPPER:
        hopper.setState(Hopper.State.EXTENDING);

        // Lift should always extend when hopper extends
        commandedLiftState = RearIntakeLift.State.EXTENDED;

        rearIntake.setState(RearIntake.State.IDLE);
        break;

      case RETRACT_HOPPER:
        // Lift should always store before hopper retracts
        commandedLiftState = RearIntakeLift.State.STORED;

        if (rearIntakeLift.isStored()) {
          hopper.setState(Hopper.State.RETRACTING);
        } else {
          hopper.setState(Hopper.State.IDLE);
        }

        rearIntake.setState(RearIntake.State.IDLE);
        break;

      case REAR_COLLECT:
        commandedLiftState = RearIntakeLift.State.EXTENDED;

        if (rearIntakeLift.isExtended()) {
          rearIntake.setState(RearIntake.State.INTAKE_COLLECT);
        } else {
          rearIntake.setState(RearIntake.State.IDLE);
        }
        break;

      case SHUFFLE_HOPPER:
        hopper.setState(Hopper.State.EXTENDING);

        // Start with lift stored, then shuffle once ready
        if (hopper.isExtended()) {
          if (rearIntakeLift.isStored() || rearIntakeLift.getState() == RearIntakeLift.State.SHUFFLE) {
            hopper.setState(Hopper.State.SHUFFLE);
            commandedLiftState = RearIntakeLift.State.SHUFFLE;
          } else {
            commandedLiftState = RearIntakeLift.State.STORED;
          }
        } else {
          commandedLiftState = RearIntakeLift.State.STORED;
        }

        rearIntake.setState(RearIntake.State.INTAKE_REVERSE);
        break;

      case REVERSE:
        hopper.setState(Hopper.State.IDLE);
        rearIntake.setState(RearIntake.State.INTAKE_REVERSE);
        break;
    }

    /* =============================
       REAR LIFT FINAL RESOLUTION
       Rear path wins when it must coordinate hopper/lift.
       Otherwise allow independent lift control.
       ============================= */
    if (commandedLiftState != null) {
      rearIntakeLift.setState(commandedLiftState);
    } else {
      switch (rearLiftWantedState) {
        case IDLE:
          rearIntakeLift.setState(RearIntakeLift.State.IDLE);
          break;

        case STORED:
          rearIntakeLift.setState(RearIntakeLift.State.STORED);
          break;

        case LIFTED:
          rearIntakeLift.setState(RearIntakeLift.State.LIFTED);
          break;

        case EXTENDED:
          rearIntakeLift.setState(RearIntakeLift.State.EXTENDED);
          break;

        case SHUFFLE:
          rearIntakeLift.setState(RearIntakeLift.State.SHUFFLE);
          break;
      }
    }

    /* =============================
       Dashboard Logging
       ============================= */
    Dashboard.logString("SuperSystem Front Wanted State", () -> frontWantedState.toString());
    Dashboard.logString("SuperSystem Rear Wanted State", () -> rearWantedState.toString());
    Dashboard.logString("SuperSystem Rear Lift Wanted State", () -> rearLiftWantedState.toString());

    Dashboard.logBoolean("ReadyToShoot", launcher::atSpeed);
    Dashboard.logBoolean("Rear Hopper Extended", hopper::isExtended);
    Dashboard.logBoolean("Rear Lift Stored", rearIntakeLift::isStored);
    Dashboard.logBoolean("Rear Lift Extended", rearIntakeLift::isExtended);
  }
}