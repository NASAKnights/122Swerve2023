package frc.robot.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.control.motors.NKTalonFX;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private NKTalonFX drive;
    private NKTalonFX turn;
    private CANCoder turnEncoder;
    private Rotation2d angleOffset = new Rotation2d();
    private SimpleMotorFeedforward feedforward;

    private int id;

    public SwerveModule(int driveMotorID, int turnMotorID, int encoderID, Rotation2d angleOffset) {
        id = (driveMotorID / 10) - 1;
        initEncoder(encoderID);
        initDriveMotor(driveMotorID);
        initTurnMotor(turnMotorID);
        this.feedforward = new SimpleMotorFeedforward(ModuleConstants.kDriveS, ModuleConstants.kDriveV, ModuleConstants.kDriveA);
        this.angleOffset = angleOffset;
    }

    public SwerveModuleState getCurrentState() {
        double velocity = getVelocityMPS();
        Rotation2d angle = getAngleRotation2d();
        return new SwerveModuleState(velocity, angle);
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDistanceMeters(), getAngleRotation2d());
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {

        double angleDelta = getCurrentState().angle.minus(desiredState.angle).getDegrees();

        if (Math.abs(angleDelta) > 90) {
            angleDelta -= 180 * Math.signum(angleDelta);
            desiredState.speedMetersPerSecond *= -1;
        }

        if (isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.kMaxSpeed;
            drive.set(ControlMode.PercentOutput, percentOutput);
        } else {
            drive.setVoltage(feedforward.calculate(desiredState.speedMetersPerSecond) + ModuleConstants.kDriveP * (desiredState.speedMetersPerSecond - getVelocityMPS()));
        }

        if (Math.abs(desiredState.speedMetersPerSecond) > (ModuleConstants.kMaxSpeed * 0.02)) {
            turn.set(ControlMode.Position, turn.getSelectedSensorPosition() - angleDelta);
        }
    }

    /**
     * 
     * @param targetAngle -180 to 180 degrees as per WPILib
     * @param unboundedAngle unbounded angle given by
     * @return
     */
    @SuppressWarnings("unused")
    private double calculateAngleDelta(double targetAngle, double unboundedAngle) {
        return boundPlusOrMinus180(targetAngle - boundPlusOrMinus180(unboundedAngle));
    }

    private static double boundPlusOrMinus180(double unboundedAngle) {
        double remainder  = (Math.abs(unboundedAngle) % 360) * Math.signum(unboundedAngle);
        if (remainder < -180) {
            remainder = remainder + 360;
        }
        else if (remainder > 180) {
            remainder = remainder - 360;
        }
        return remainder;
    }

    private void initDriveMotor(int driveMotorID) {
        drive = new NKTalonFX(driveMotorID);

        drive.configFactoryDefault();
        drive.configAllSettings(ModuleConstants.kDriveMotorConfig);
        drive.setInverted(ModuleConstants.kDriveMotorInverted);
        drive.setNeutralMode(ModuleConstants.kDriveMotorNeutral);
        drive.setSelectedSensorPosition(0);

    }

    private void initTurnMotor(int turnMotorID) {
        turn = new NKTalonFX(turnMotorID);
        turn.configFactoryDefault();
        turn.configAllSettings(ModuleConstants.kTurnMotorConfig);
        turn.setInverted(ModuleConstants.kTurnMotorInverted);
        turn.setSensorPhase(true);
        turn.setNeutralMode(ModuleConstants.kTurnMotorNeutral);
        turn.configRemoteFeedbackFilter(turnEncoder, 0);
        turn.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, 0);
        // double absPos = turnEncoder.getAbsolutePosition() - angleOffset.getDegrees();
        // turn.setSelectedSensorPosition(Conversions.degreesToFalcon(absPos, Constants.TURN_GEAR_RATIO));
    }

    private void initEncoder(int encoderID) {
        turnEncoder = new CANCoder(encoderID);

        turnEncoder.configFactoryDefault();
        turnEncoder.configAllSettings(ModuleConstants.kEncoderConfig);
    }

    public void updateSmartDash() {
        SmartDashboard.putNumber(id + " Module Encoder Raw Position", turnEncoder.getPosition());
        SmartDashboard.putNumber(id + " Motor Selected Sensor Position", turn.getSelectedSensorPosition());
        SmartDashboard.putNumber(id + " Module Angle", getAngleRotation2d().getDegrees());
    }

    public Rotation2d getAngleRotation2d() {
        return Rotation2d.fromDegrees(boundPlusOrMinus180(turnEncoder.getPosition() - angleOffset.getDegrees()));
    }

    public double getDistanceMeters() {
        return 0.0;
    }

    public double getVelocityMPS() {
        return Conversions.falconToMPS(drive.getSelectedSensorVelocity(), ModuleConstants.kWheelCircumference, ModuleConstants.kDriveGearRatio);
    }

    public void testDriveSpinny(double output) {
        drive.set(output);
    }

    public void testTurnSpinny(double output) {
        turn.set(output);
    }

    public void testStopSpinny() {
        drive.set(0);
        turn.set(0);
    }

    public void setDriveVoltage(double voltage) {
        drive.setVoltage(voltage);
    }

    public static final class Conversions {
        public static double falconToDegrees(double counts, double gearRatio) {
            // ratio = motor/wheel
            return counts * (360.0 / (gearRatio * 2048.0));
        }

        public static double degreesToFalcon(double degrees, double gearRatio) {
            double ticks = degrees / (360.0 / (gearRatio * 2048.0));
            return ticks;
        }

        /**
         * Converts a falcon motor position into distance traveled
         * @param falconPosition falcon position sensor counts
         * @param circumference wheel circumference in meters
         * @param gearRatio  motor rotations/wheel rotations
         * @return distance traveled in meters
         */
        public static double falconToMeters(double falconPosition, double circumference, double gearRatio) {
            double motorRotations = falconPosition / 2048.0;
            // rotations * m * g, where g is in units motor/wheel rotation
            double distance = motorRotations * circumference / gearRatio;
            return distance;
        }

        public static double falconToRPM(double velocityCounts, double gearRatio) {
            double motorRPM = velocityCounts * (600.0 / 2048.0);
            double mechRPM = motorRPM / gearRatio;
            return mechRPM;
        }

        public static double RPMToFalcon(double RPM, double gearRatio) {
            double motorRPM = RPM * gearRatio;
            double sensorCounts = motorRPM * (2048.0 / 600.0);
            return sensorCounts;
        }

        public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
            double wheelRPM = falconToRPM(velocitycounts, gearRatio);
            double wheelMPS = (wheelRPM * circumference) / 60;
            return wheelMPS;
        }

        public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
            double wheelRPM = ((velocity * 60) / circumference);
            double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
            return wheelVelocity;
        }
    }
}
