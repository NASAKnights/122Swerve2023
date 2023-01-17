package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

public class SwerveDrive extends SubsystemBase {

    private AHRS navx;

    private ChassisSpeeds speeds;
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;

    private SwerveModule frontLeft, frontRight, backLeft, backRight;
    private SwerveModule[] modules;

    public SwerveDrive(AHRS navx) {
        this.navx = navx;
        this.navx.calibrate();

        this.speeds = new ChassisSpeeds();
        this.kinematics = new SwerveDriveKinematics(
                // Front left
                new Translation2d(Constants.kTrackwidthMeters / 2.0,
                                Constants.kWheelbaseMeters / 2.0),
                // Front right
                new Translation2d(Constants.kTrackwidthMeters / 2.0,
                                -Constants.kWheelbaseMeters / 2.0),
                // Back left
                new Translation2d(-Constants.kTrackwidthMeters / 2.0,
                                Constants.kWheelbaseMeters / 2.0),
                // Back right
                new Translation2d(-Constants.kTrackwidthMeters / 2.0,
                                -Constants.kWheelbaseMeters / 2.0));

        this.frontLeft = new SwerveModule(
                Constants.kFrontLeftDriveMotorID,
                Constants.kFrontLeftTurnMotorID,
                Constants.kFrontLeftEncoderID,
                Constants.kFrontLeftOffset);

        this.frontRight = new SwerveModule(
                Constants.kFrontRightDriveMotorID,
                Constants.kFrontRightTurnMotorID,
                Constants.kFrontRightEncoderID,
                Constants.kFrontRightOffset);

        this.backLeft = new SwerveModule(
                Constants.kBackLeftDriveMotorID,
                Constants.kBackLeftTurnMotorID,
                Constants.kBackLeftEncoderID,
                Constants.kBackLeftOffset);

        this.backRight = new SwerveModule(
                Constants.kBackRightDriveMotorID,
                Constants.kBackRightTurnMotorID,
                Constants.kBackRightEncoderID,
                Constants.kBackRightOffset);

                this.modules = new SwerveModule[] { this.frontLeft, this.frontRight, this.backLeft, this.backRight };

                this.odometry = new SwerveDriveOdometry(this.kinematics, this.getHeading(), this.getModulePositions());
    }

    public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
        this.speeds = speeds;
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        // states[0].speedMetersPerSecond *= -1;

        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.kMaxSpeed);
        
        for (int i = 0; i < 4; i++) {
            this.modules[i].setDesiredState(states[i], isOpenLoop);
        }
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-navx.getAngle());
    }

    public void resetHeading() {
        navx.zeroYaw();
    }

    public void updateSmartDash() {
        for (SwerveModule module : this.modules) {
                module.updateSmartDash();
        }
        SmartDashboard.putNumber("heading", this.getHeading().getDegrees());
        SmartDashboard.putNumber("x", this.odometry.getPoseMeters().getX());
        SmartDashboard.putNumber("y", this.odometry.getPoseMeters().getY());
    }

    public SwerveModule[] getModules() {
        return this.modules;
    }

    private SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            this.frontLeft.getPosition(),
            this.frontRight.getPosition(),
            this.backLeft.getPosition(),
            this.backRight.getPosition()
        };
    }

    public void resetPose(Pose2d position) {
        this.odometry.resetPosition(this.getHeading(), this.getModulePositions(), position);
    }

    private void updateOdometry() {
        // SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        this.odometry.update(this.getHeading(), this.getModulePositions());
    }
    @Override
    public void periodic() {
        this.updateOdometry();
        this.updateSmartDash();
    }

}
