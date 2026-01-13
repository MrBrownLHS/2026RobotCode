package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.utilities.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Command that reads joystick (or other) suppliers and drives the swerve
 * subsystem. This class performs input deadbanding, slew-rate limiting,
 * converts the 2D translation vector into a magnitude+angle pair, scales
 * values to physical speeds, and calls the SwerveSubsystem's drive method.

 */
public class SwerveController extends Command {
    
    private Swerve swerveSubsystem;

    // Input suppliers (usually joystick axes or lambda-wrapped inputs)
    
    
    
    
   
    private DoubleSupplier translationSupplier;// - translationSupplier: forward/backwards input
    private DoubleSupplier strafeSupplier;// - strafeSupplier: left/right input
    private DoubleSupplier rotationSupplier;// - rotationSupplier: rotational input (yaw)
    private BooleanSupplier robotCentricSupplier;// - robotCentricSupplier: boolean indicating whether to use robot-centric 
    //   controls (true) or field-centric controls (false)

    // Slew rate limiters prevent sudden jumps in commanded speed. The value
    // passed in is the rate (units per second) the value is allowed to change.
    // Tuning these affects perceived joystick responsiveness and jerk.
    private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);

    /**
     * Constructor — wires suppliers and declares subsystem requirements.
     *
     * @param swerveSubsystem subsystem to control
     * @param translationSupplier forward/backwards input supplier
     * @param strafeSupplier left/right input supplier
     * @param rotationSupplier rotational input supplier
     * @param robotCentricSupplier whether to use robot-centric control
     */
    public SwerveController(Swerve swerveSubsystem, DoubleSupplier translationSupplier, DoubleSupplier strafeSupplier, DoubleSupplier rotationSupplier, BooleanSupplier robotCentricSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.translationSupplier = translationSupplier;
        this.strafeSupplier = strafeSupplier;
        this.rotationSupplier = rotationSupplier;
        this.robotCentricSupplier = robotCentricSupplier;

        // Ensure this command requires the swerve subsystem so other commands
        // cannot run simultaneously and conflict with drive outputs.
        addRequirements(swerveSubsystem);
    }

    /**
     * Called repeatedly while the command is scheduled. Reads raw inputs,
     * filters and scales them, then forwards a velocity+rotation command to
     * the swerve subsystem.
     */
    @Override
    public void execute() {
        // Read raw translation/strafe from suppliers and apply slew limiting
        // to smooth sudden changes. The limiter returns a rate-constrained
        // version of the input value.
        double translationValue = translationLimiter.calculate(translationSupplier.getAsDouble());
        double strafeValue = strafeLimiter.calculate(strafeSupplier.getAsDouble());

        // Pack the two linear axes into a Translation2d representing the
        // desired planar linear velocity vector (units: controller axis range)
        Translation2d linearVelocity = new Translation2d(translationValue, strafeValue);

        // Compute the magnitude (speed) of the translation vector then apply
        // a deadband to ignore small joystick noise. The deadband is taken
        // from Constants.DriverConstants.kDeadband. Squaring the magnitude
        // provides finer low-speed control (makes input less sensitive
        // near zero while preserving sign via the angle separately).
        double linearMagnitude = MathUtil.applyDeadband(linearVelocity.getNorm(), Constants.DriverConstants.kDeadband);
        linearMagnitude *= linearMagnitude; // square for finer low-speed control

        // Determine the angle (direction) of the translation vector. If the
        // joystick is essentially centered, default to 0 radians to avoid
        // divide-by-zero or NaN from angle computations.
        Rotation2d angle = null;

        if (linearVelocity.getNorm() > 0.001) { // Prevents divide by zero error
            angle = linearVelocity.getAngle();
        } else {
            angle = new Rotation2d(0.0);
        }

        // Reconstruct a translation vector whose magnitude is the squared and
        // deadbanded linearMagnitude and whose angle is the previously
        // determined direction. The sequence below composes a Pose2d at the
        // origin with the desired rotation, then transforms it by a
        // Transform2d that moves it forward by linearMagnitude. Finally we
        // extract the Translation2d component which is the linear velocity
        // vector we will send to the drivetrain (in controller axis units).
        linearVelocity = new Pose2d(
            new Translation2d(), 
            angle
        ).transformBy(
            new Transform2d(
                linearMagnitude, 
                0.0, 
                new Rotation2d()
            )
        ).getTranslation();

        // Read rotation input, apply deadband, and pass through slew limiter
        // to smooth rotational commands as well.
        double rotationValue = rotationLimiter.calculate(MathUtil.applyDeadband(rotationSupplier.getAsDouble(), Constants.DriverConstants.kDeadband));

        // Scale the normalized controller linear and angular values to the
        // physical maximum speeds defined in constants and send the command
        // to the swerve subsystem. The third parameter in drive() indicates
        // whether to use field-relative control; here we invert the
        // robotCentricSupplier because the supplier gives robot-centric when
        // true but the method expects field-relative when true (hence '!').
        swerveSubsystem.drive(
            linearVelocity.times(Constants.SwerveConstants.PhysicalMaxTranslationSpeed), // linear velocity (m/s)
            rotationValue * Constants.SwerveConstants.PhysicalMaxAngularVelocity, // angular velocity (rad/s)
            !robotCentricSupplier.getAsBoolean(), // fieldRelative: invert robot-centric flag
            true // open-loop flag (true = open-loop control), left as original behavior
        );
    }
}