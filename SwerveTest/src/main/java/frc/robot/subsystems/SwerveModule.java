package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Basic SwerveModule using REV CANSparkMax for drive and steer and CTRE CANCoder for absolute heading.
 * This implementation uses simple open-loop control for drive and a P-controller for steering position.
 * Replace gains and CAN IDs with values matching your robot.
 */
public class SwerveModule {
  private final CANSparkMax driveMotor;
  private final CANSparkMax steerMotor;
  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder steerEncoder;
  private final CANCoder cancoder;

  // Physical constants - tune for your hardware
  private static final double WHEEL_DIAMETER_METERS = 0.0762; // 3 in
  private static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER_METERS;
  private static final double DRIVE_GEAR_RATIO = 6.86; // example
  private static final double STEER_GEAR_RATIO = 150.0/7.0; // example
  private static final double NEO_FREE_SPEED_RPM = 5676.0;

  // simple P gain for steering (tune as needed)
  private static final double STEER_P = 0.8;

  private final double cancoderOffsetDeg;

  public SwerveModule(int driveCanId, int steerCanId, double cancoderOffsetDeg) {
    this.driveMotor = new CANSparkMax(driveCanId, MotorType.kBrushless);
    this.steerMotor = new CANSparkMax(steerCanId, MotorType.kBrushless);
    this.driveEncoder = driveMotor.getEncoder();
    this.steerEncoder = steerMotor.getEncoder();

    // CANcoder assumed to be at steerCanId + 100 (or set appropriately); the user should set real CAN ID.
    // For reliability, the user should supply the CANCoder ID; here we use steerCanId + 100 as a placeholder.
    this.cancoder = new CANCoder(steerCanId + 100);

    this.cancoderOffsetDeg = cancoderOffsetDeg;

    // Basic motor settings - user may want to tune or apply factory defaults
    driveMotor.restoreFactoryDefaults();
    steerMotor.restoreFactoryDefaults();

    // Burn configs if desired
  }

  /** Convert encoder position (rotations) to module angle Rotation2d */
  private Rotation2d getSteerAngle() {
    // steerEncoder returns motor rotations; convert to wheel angle using gear ratio
    double motorRotations = steerEncoder.getPosition();
    double wheelRotations = motorRotations / STEER_GEAR_RATIO;
    double angleRad = wheelRotations * 2.0 * Math.PI;
    return new Rotation2d(angleRad);
  }

  /** Read absolute angle from CANCoder and apply offset */
  public Rotation2d getAbsoluteAngle() {
    double angle = cancoder.getAbsolutePosition(); // degrees
    angle -= cancoderOffsetDeg;
    return Rotation2d.fromDegrees(angle);
  }

  /** Set desired module state (speed in m/s and angle). */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize state to avoid unnecessary rotation
    Rotation2d currentAngle = getSteerAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentAngle);

    // Drive: convert speed (m/s) to percent output using theoretical max RPM
    double wheelRotPerSec = state.speedMetersPerSecond / WHEEL_CIRCUMFERENCE;
    double motorRPM = wheelRotPerSec * 60.0 * DRIVE_GEAR_RATIO; // motor RPM
    double drivePercent = motorRPM / NEO_FREE_SPEED_RPM;
    driveMotor.set(drivePercent);

    // Steer: simple P-control on angle error
    double targetAngleRad = state.angle.getRadians();
    double currentAngleRad = currentAngle.getRadians();
    double error = normalizeAngle(targetAngleRad - currentAngleRad);
    double steerOutput = STEER_P * error;
    steerMotor.set(steerOutput);
  }

  /** Called periodically to allow closed-loop updates if needed */
  public void periodic() {
    // In this simple implementation, steer is controlled open-loop by setDesiredState.
    // You could implement velocity or position control using SparkMax PID controllers here.
  }

  private static double normalizeAngle(double angleRad) {
    while (angleRad > Math.PI) {
      angleRad -= 2.0 * Math.PI;
    }
    while (angleRad < -Math.PI) {
      angleRad += 2.0 * Math.PI;
    }
    return angleRad;
  }
}
