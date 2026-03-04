package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;




import frc.robot.utilities.Constants;
import frc.robot.utilities.Dashboard;

public class Swerve extends SubsystemBase {
    private final Pigeon2 gyroscope;
    private final StatusSignal<Angle> gyroAngle;

    private SwerveDriveOdometry swerveOdometry;
    private SwerveDriveKinematics driveKinematics;

    private NEOSwerveModule[] swerveModules; // Change this to FXSwerveModule[] if you are using the FXSwerveModule class

    //private SlewRateLimiter translationLimiter = new SlewRateLimiter(2.9);
    //private SlewRateLimiter strafeLimiter = new SlewRateLimiter(2.9);
    //private SlewRateLimiter rotationLimiter = new SlewRateLimiter(2.9);

    private Field2d field;

    private ChassisSpeeds commandedSpeeds = new ChassisSpeeds();
    private boolean fieldRelativeLast = false;
    private boolean openLoopLast = false;

  

    public Swerve() {
        gyroscope = new Pigeon2(Constants.gyroID);
        gyroAngle = gyroscope.getYaw();
        BaseStatusSignal.setUpdateFrequencyForAll(50.0, gyroAngle);
        resetHeading();

       

        swerveModules = new NEOSwerveModule[] {
            new NEOSwerveModule(0, Constants.ModuleConstants.FrontLeftModule.constants),
            new NEOSwerveModule(1, Constants.ModuleConstants.FrontRightModule.constants),
            new NEOSwerveModule(2, Constants.ModuleConstants.BackLeftModule.constants),
            new NEOSwerveModule(3, Constants.ModuleConstants.BackRightModule.constants)
        };

        driveKinematics = Constants.SwerveConstants.SwerveKinematics;
        swerveOdometry = new SwerveDriveOdometry(driveKinematics, getYawRotation2d(), getSwerveModulePositions());
        field = new Field2d();
        resetModulesToAbsolute();

        // swerveTab.addString("Swerve Module States", () -> {
        //     StringBuilder statesString = new StringBuilder();
        //     for(NEOSwerveModule module : swerveModules) {
        //         SwerveModuleState state = module.getSwerveModuleState();
        //         statesString.append(String.format("Module %d: Angle=%.2f deg, Speed=%.2f m/s\n", module.moduleNumber, state.angle.getDegrees(), state.speedMetersPerSecond));
        //     }
        //     return statesString.toString();
        // });
    }

    // public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    //     SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(fieldRelative
    //         ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYawRotation2d())
    //         : new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
    //     );

    //     SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxTranslationSpeed);
    //     for(NEOSwerveModule module : swerveModules) {
    //         module.setDesiredState(swerveModuleStates[module.moduleNumber], false);
    //     }
    // }

    // Updated drive method to store last command for dashboard logging added 3/4
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

    fieldRelativeLast = fieldRelative;
    openLoopLast = isOpenLoop;

    commandedSpeeds = fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(
            translation.getX(),
            translation.getY(),
            rotation,
            getYawRotation2d())
        : new ChassisSpeeds(
            translation.getX(),
            translation.getY(),
            rotation
        );

    SwerveModuleState[] swerveModuleStates =
        driveKinematics.toSwerveModuleStates(commandedSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates,
        Constants.SwerveConstants.PhysicalMaxTranslationSpeed
    );

    for (NEOSwerveModule module : swerveModules) {
        module.setDesiredState(swerveModuleStates[module.moduleNumber], false);
    }
}

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] swerveModuleStates = driveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.PhysicalMaxTranslationSpeed);
        setSwerveModuleStates(swerveModuleStates);
    }

    public SwerveModuleState[] getSwerveModuleStates() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for(NEOSwerveModule module : swerveModules) {
            swerveModuleStates[module.moduleNumber] = module.getSwerveModuleState();
        }

        return swerveModuleStates;
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for(NEOSwerveModule module : swerveModules) {
            swerveModulePositions[module.moduleNumber] = module.getSwerveModulePosition();
        }

        return swerveModulePositions;
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return driveKinematics.toChassisSpeeds(getSwerveModuleStates());
    }

    public void setSwerveModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.PhysicalMaxTranslationSpeed);
        for(NEOSwerveModule swerveModule : swerveModules) {
            swerveModule.setDesiredState(desiredStates[swerveModule.moduleNumber], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public double getHeading() {
        return MathUtil.inputModulus(getRawHeading(), 0, 360);
    }

    public Rotation2d getYawRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public double getRawHeading() {
        return gyroAngle.getValueAsDouble();
    }

    public void resetSwerveOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYawRotation2d(), getSwerveModulePositions(), pose);
    }

    public void resetModulesToAbsolute() {
        for(NEOSwerveModule swerveModule : swerveModules) {
            swerveModule.resetToAbsolute();
        }
    }

    public void resetHeading() {
        gyroscope.setYaw(0.0);
    }

    public void stop() {
        for(NEOSwerveModule swerveModule : swerveModules) {
            swerveModule.stop();
        }
    }


//Autonomous Commands - duplicate and rename as needed to create new commands based on field position. Add selections to SmartDashboard.
   
    public Command swerveAuto(double xMeters, double yMeters, double speed, double rotationSpeed) {
        return new SequentialCommandGroup(
            new RunCommand(() -> drive(
                new Translation2d(speed * Math.signum(xMeters), speed * Math.signum(yMeters)),
                rotationSpeed, // Allows rotation while moving
                true, false),
                this
            ).until(() -> hasReachedDistance(xMeters, yMeters)),
            
            stopSwerveCommand() // Stops the robot after reaching distance
        );
    }
    
    private boolean hasReachedDistance(double xTarget, double yTarget) {
        double currentX = getPose().getX();
        double currentY = getPose().getY();
        return Math.abs(currentX - xTarget) < 0.05 && Math.abs(currentY - yTarget) < 0.05;
    }
    
    public Command stopSwerveCommand() {
        return new InstantCommand(() -> drive(new Translation2d(0, 0), 0, true, false), this);
    }



    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll(gyroAngle);
        swerveOdometry.update(getYawRotation2d(), getSwerveModulePositions());
        field.setRobotPose(getPose());

        double[] measuredStates = {
                swerveModules[0].getSwerveModuleState().angle.getDegrees(),
                swerveModules[0].getSwerveModuleState().speedMetersPerSecond,
                swerveModules[1].getSwerveModuleState().angle.getDegrees(),
                swerveModules[1].getSwerveModuleState().speedMetersPerSecond,
                swerveModules[2].getSwerveModuleState().angle.getDegrees(),
                swerveModules[2].getSwerveModuleState().speedMetersPerSecond,
                swerveModules[3].getSwerveModuleState().angle.getDegrees(),
                swerveModules[3].getSwerveModuleState().speedMetersPerSecond,
        };

        double[] desiredStates = {
                swerveModules[0].getDesiredState().angle.getDegrees(),
                swerveModules[0].getDesiredState().speedMetersPerSecond,
                swerveModules[1].getDesiredState().angle.getDegrees(),
                swerveModules[1].getDesiredState().speedMetersPerSecond,
                swerveModules[2].getDesiredState().angle.getDegrees(),
                swerveModules[2].getDesiredState().speedMetersPerSecond,
                swerveModules[3].getDesiredState().angle.getDegrees(),
                swerveModules[3].getDesiredState().speedMetersPerSecond,

        };

        BaseStatusSignal.refreshAll(gyroAngle);
        swerveOdometry.update(getYawRotation2d(), getSwerveModulePositions());
        field.setRobotPose(getPose());

        Pose2d pose = getPose();
        ChassisSpeeds measuredSpeeds = getRobotRelativeSpeeds();

        // ===============================
        // 🔹 Robot Pose
        // ===============================

        Dashboard.logPose2d("Swerve/Pose", () -> pose);
        Dashboard.logDouble("Swerve/Pose/X", pose::getX);
        Dashboard.logDouble("Swerve/Pose/Y", pose::getY);
        Dashboard.logDouble("Swerve/Pose/HeadingDeg",
            () -> pose.getRotation().getDegrees());

        // ===============================
        // 🔹 Gyro
        // ===============================

        Dashboard.logDouble("Swerve/Gyro/RawYaw", this::getRawHeading);
        Dashboard.logDouble("Swerve/Gyro/WrappedHeading", this::getHeading);

        // ===============================
        // 🔹 Chassis Speeds (Measured)
        // ===============================

        Dashboard.logDouble("Swerve/Measured/Vx",
            () -> measuredSpeeds.vxMetersPerSecond);

        Dashboard.logDouble("Swerve/Measured/Vy",
            () -> measuredSpeeds.vyMetersPerSecond);

        Dashboard.logDouble("Swerve/Measured/Omega",
            () -> measuredSpeeds.omegaRadiansPerSecond);

        // ===============================
        // 🔹 Commanded Speeds
        // ===============================

        Dashboard.logDouble("Swerve/Commanded/Vx",
            () -> commandedSpeeds.vxMetersPerSecond);

        Dashboard.logDouble("Swerve/Commanded/Vy",
            () -> commandedSpeeds.vyMetersPerSecond);

        Dashboard.logDouble("Swerve/Commanded/Omega",
            () -> commandedSpeeds.omegaRadiansPerSecond);

        Dashboard.logBoolean("Swerve/FieldRelative",
            () -> fieldRelativeLast);

        Dashboard.logBoolean("Swerve/OpenLoop",
            () -> openLoopLast);

        // ===============================
        // 🔹 Module Telemetry
        // ===============================

        for (NEOSwerveModule module : swerveModules) {

            int i = module.moduleNumber;

            SwerveModuleState measured = module.getSwerveModuleState();
            SwerveModuleState desired = module.getDesiredState();

            Dashboard.logDouble(
                "Swerve/Module" + i + "/MeasuredAngle",
                () -> measured.angle.getDegrees());

            Dashboard.logDouble(
                "Swerve/Module" + i + "/DesiredAngle",
                () -> desired.angle.getDegrees());

            Dashboard.logDouble(
                "Swerve/Module" + i + "/AngleError",
                () -> desired.angle.minus(measured.angle).getDegrees());

            Dashboard.logDouble(
                "Swerve/Module" + i + "/MeasuredSpeed",
                () -> measured.speedMetersPerSecond);

            Dashboard.logDouble(
                "Swerve/Module" + i + "/DesiredSpeed",
                () -> desired.speedMetersPerSecond);

            Dashboard.logDouble(
                "Swerve/Module" + i + "/SpeedError",
                () -> desired.speedMetersPerSecond
                    - measured.speedMetersPerSecond);
        }
    }
}