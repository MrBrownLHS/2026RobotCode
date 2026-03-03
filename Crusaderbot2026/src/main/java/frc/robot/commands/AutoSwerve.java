// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoSwerve extends Command {
    private final Swerve swerve;
    private final double targetX; // meters relative to start
    private final double targetY; // meters relative to start
    private final double maxSpeed; // m/s
    private final double slowDownDistance; // meters to start slowing down
    private final Double targetHeadingDegrees; // null if fixed rotationSpeed is used
    private Pose2d startPose;

    // PID controllers for X, Y movement, and rotation
    private final PIDController xController = new PIDController(2.0, 0.0, 0.0); // tune
    private final PIDController yController = new PIDController(2.0, 0.0, 0.0); // tune
    private final PIDController rotationController = new PIDController(1.5, 0.0, 0.0); // tune

    /**
     * Use this constructor for fixed rotation speed (legacy behavior)
     */
    public AutoSwerve(Swerve swerve, double xMeters, double yMeters,
                      double maxSpeed, double rotationSpeed, double slowDownDistance) {
        this.swerve = swerve;
        this.targetX = xMeters;
        this.targetY = yMeters;
        this.maxSpeed = maxSpeed;
        this.slowDownDistance = slowDownDistance;
        this.targetHeadingDegrees = null; // no angle control
        rotationController.disableContinuousInput(); // not used
        addRequirements(swerve);
    }

    /**
     * Use this constructor to drive to a distance and rotate to a specific heading in degrees
     */
    public AutoSwerve(Swerve swerve, double xMeters, double yMeters,
                      double maxSpeed, double targetHeadingDegrees, double slowDownDistance, boolean useHeading) {
        this.swerve = swerve;
        this.targetX = xMeters;
        this.targetY = yMeters;
        this.maxSpeed = maxSpeed;
        this.slowDownDistance = slowDownDistance;
        this.targetHeadingDegrees = targetHeadingDegrees;
        rotationController.enableContinuousInput(-180.0, 180.0); // wrap around 360 degrees
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        startPose = swerve.getPose();
        xController.setTolerance(0.02);
        yController.setTolerance(0.02);
        rotationController.setTolerance(1.0); // degrees tolerance
    }

    @Override
    public void execute() {
        double currentX = swerve.getPose().getX() - startPose.getX();
        double currentY = swerve.getPose().getY() - startPose.getY();

        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;

        double distanceToTarget = Math.hypot(deltaX, deltaY);

        // Scale speed down as we approach the target
        double speedScale = MathUtil.clamp(distanceToTarget / slowDownDistance, 0.1, 1.0);

        double xSpeed = MathUtil.clamp(xController.calculate(currentX, targetX), -maxSpeed, maxSpeed) * speedScale;
        double ySpeed = MathUtil.clamp(yController.calculate(currentY, targetY), -maxSpeed, maxSpeed) * speedScale;

        double rotSpeed;
        if (targetHeadingDegrees != null) {
            // Calculate rotation PID output from the swerve pose's rotation (degrees)
            double currentHeading = swerve.getPose().getRotation().getDegrees(); // degrees
            rotSpeed = rotationController.calculate(currentHeading, targetHeadingDegrees);
        } else {
            rotSpeed = 0.0; // fallback to no rotation control
        }

        swerve.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, true, false);
    }

    @Override
    public boolean isFinished() {
        boolean positionReached = xController.atSetpoint() && yController.atSetpoint();
        boolean rotationReached = targetHeadingDegrees == null || rotationController.atSetpoint();
        return positionReached && rotationReached;
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new Translation2d(0, 0), 0.0, true, false);
    }
}