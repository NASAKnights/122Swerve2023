package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;

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

        this.modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };
    }

    public void drive(ChassisSpeeds speeds, boolean isOpenLoop) {
        this.speeds = speeds;
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        // states[0].speedMetersPerSecond *= -1;

        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.kMaxSpeed);
        
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(states[i], isOpenLoop);
        }
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-navx.getAngle());
    }

    public void resetHeading() {
        navx.zeroYaw();
    }

    public void updateSmartDash() {
        for (SwerveModule module : modules) {
                module.updateSmartDash();
        }
        SmartDashboard.putNumber("heading", navx.getAngle());
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    private void updateOdometry() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];

        for (int i = 0; i < modules.length; i++) {
            positions[i] = new SwerveModulePosition(modules[i].getDistanceMeters(), modules[i].getAngleRotation2d());
        }
        odometry.update(getHeading(), null);
    }

}
