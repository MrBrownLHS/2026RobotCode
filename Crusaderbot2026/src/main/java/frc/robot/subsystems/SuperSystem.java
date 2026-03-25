// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.Dashboard;
//import frc.robot.utilities.DriverHUD;

public class SuperSystem extends SubsystemBase {
  /** Creates a new SuperSystem. */
  public enum WantedState {
    IDLE,
    FRONT_COLLECT,
    LAUNCH_FAR,
    LAUNCH_CLOSE,
    YEET_PASS,
    RETRACT_HOPPER,
    EXTEND_HOPPER,
    REAR_COLLECT,
    SHUFFLE_HOPPER,
    REVERSE
  }

  private WantedState wantedState = WantedState.IDLE;

  private final Launcher launcher;
  private final RearIntake rearIntake;
  private final FrontIntake intake;
  private final Hopper hopper;
  private final Agitator agitator;
  private final RearIntakeLift rearIntakeLift;

  public SuperSystem(Launcher launcher, RearIntake rearIntake, FrontIntake intake, Hopper hopper, Agitator agitator, RearIntakeLift rearIntakeLift) {
    this.launcher = launcher;
    this.rearIntake = rearIntake;
    this.intake = intake;
    this.hopper = hopper;
    this.agitator = agitator;
    this.rearIntakeLift = rearIntakeLift;
  }

  public void setWantedState(WantedState newWantedState) {
    wantedState = newWantedState;
  }

  /** Expose the current wanted state for dashboards and commands. */
  public WantedState getWantedState() {
    return wantedState;
  }

  public boolean isLauncheratSpeed() {
    return launcher.atSpeed();
  }

  @Override
  public void periodic() {
      switch (wantedState) {
        case IDLE:
          launcher.setState(Launcher.State.IDLE);
          rearIntake.setState(RearIntake.State.IDLE);
          intake.setState(FrontIntake.State.IDLE);
          hopper.setState(Hopper.State.IDLE);
          agitator.setState(Agitator.State.IDLE);
          rearIntakeLift.setState(RearIntakeLift.State.IDLE);

          break;

        case FRONT_COLLECT:
          launcher.setState(Launcher.State.LAUNCH_COLLECT);

          if (launcher.atSpeed()) {
              intake.setState(FrontIntake.State.INTAKE_COLLECT);
              agitator.setState(Agitator.State.IDLE); 
          }
            break;
          

        case LAUNCH_FAR:
          
              launcher.setState(Launcher.State.LAUNCH_FAR);

          if (launcher.atSpeed()) {
              intake.setState(FrontIntake.State.INTAKE_LAUNCH);
              agitator.setState(Agitator.State.AGITATE);
         
              }
              break;

        case LAUNCH_CLOSE:

            launcher.setState(Launcher.State.LAUNCH_CLOSE);

            if (launcher.atSpeed()) {
                intake.setState(FrontIntake.State.INTAKE_LAUNCH);
                agitator.setState(Agitator.State.AGITATE);  
            }
            break;
        
        case YEET_PASS:

            launcher.setState(Launcher.State.YEET_PASS);

            if (launcher.atSpeed()) {
              intake.setState(FrontIntake.State.INTAKE_LAUNCH);
            }
            break;

        case RETRACT_HOPPER:
          rearIntakeLift.setState(RearIntakeLift.State.STORED);

          if (rearIntakeLift.isStored()) {
            hopper.setState(Hopper.State.RETRACTING);
          } else {
            hopper.setState(Hopper.State.IDLE);
          }
          break;

case EXTEND_HOPPER:
          hopper.setState(Hopper.State.EXTENDING);

          if (hopper.isExtended()) {
            rearIntakeLift.setState(RearIntakeLift.State.EXTENDED);
          } else {
            rearIntakeLift.setState(RearIntakeLift.State.IDLE);
          }
          break;
        
        case REAR_COLLECT:

            hopper.setState(Hopper.State.EXTENDING);
            rearIntakeLift.setState(RearIntakeLift.State.EXTENDED);

            // Check ACTUAL POSITION, not state
            if (hopper.isExtended() && rearIntakeLift.isExtended()) {
                rearIntake.setState(RearIntake.State.INTAKE_COLLECT);
            }

            break;

        case SHUFFLE_HOPPER:
            hopper.setState(Hopper.State.EXTENDING);
            rearIntakeLift.setState(RearIntakeLift.State.STORED);
            
            if (hopper.isExtended() && rearIntakeLift.isStored()) {
              hopper.setState(Hopper.State.SHUFFLE);
              rearIntakeLift.setState(RearIntakeLift.State.SHUFFLE);
            }
            
            break;

        case REVERSE:
          intake.setState(FrontIntake.State.INTAKE_REVERSE);
          agitator.setState(Agitator.State.REVERSE);
          rearIntake.setState(RearIntake.State.INTAKE_REVERSE);
          break;
      }
      Dashboard.logString("SuperSystem Wanted State", () -> wantedState.toString());
      // Aggregated readiness flags
      Dashboard.logBoolean("ReadyToShoot", () -> launcher.atSpeed());
      Dashboard.logBoolean("ReadyToCollect", () -> intake.getState() == FrontIntake.State.INTAKE_COLLECT);
    //   DriverHUD.logString("SuperSystem State", () -> wantedState.toString());
    // // Also publish the ready flags to the driver HUD for live SmartDashboard display
    //   DriverHUD.logReadyFlags(launcher, intake);
  }
}
