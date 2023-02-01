package frc.robot;

import static frc.robot.Constants.kDriverPort;
import static frc.robot.Constants.kNavXPort;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.ModuleTest;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.drive.commands.ModuleTestCommand;
import frc.robot.drive.commands.TestModuleAngleCommand;
import frc.robot.drive.commands.VelocityTest;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;
    private SwerveDrive swerve;
    // private ModuleTest test;

    public RobotContainer() {
        driver = new Joystick(kDriverPort);

        navx = new AHRS(kNavXPort);

        // test = new ModuleTest(new SwerveModule(Constants.kFrontRightDriveMotorID,Constants.kFrontRightTurnMotorID, 
                        // Constants.kFrontRightEncoderID, Rotation2d.fromDegrees(150.654)), Constants.kFrontRightPosition);

        // test = new ModuleTest(new SwerveModule(Constants.kFrontLeftDriveMotorID,Constants.kFrontLeftTurnMotorID, 
                        // Constants.kFrontLeftEncoderID, Rotation2d.fromDegrees(358.188)), Constants.kFrontLeftPosition);

        // test = new ModuleTest(new SwerveModule(Constants.kBackLeftDriveMotorID,Constants.kBackLeftTurnMotorID, 
                        // Constants.kBackLeftEncoderID, Rotation2d.fromDegrees(257.607)), Constants.kBackLeftPosition);
        // test = new ModuleTest(new SwerveModule(Constants.kBackRightDriveMotorID,Constants.kBackRightTurnMotorID, 
                        // Constants.kBackRightEncoderID, Rotation2d.fromDegrees(103.359)), Constants.kBackRightPosition);
        swerve = new SwerveDrive(navx);
        swerve.readoffsets();
        swerve.initDashboard();

        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        // swerve.setDefaultCommand(new DriveCommand(driver, swerve));
        swerve.setDefaultCommand(new VelocityTest(swerve, 0.0));
        // test.setDefaultCommand(new ModuleTestCommand(driver, test));
    }

    private void configureButtonBindings() {
        new JoystickButton(driver, 1).onTrue(new InstantCommand(swerve::resetHeading));
        
    }

    public void periodic() {
        swerve.updateSmartDash();
        swerve.writeOffsets();
        swerve.readoffsets();
        // test.updateSmartDash();
        swerve.initDashboard();
        // SmartDashboard.putNumber("Module Velocity", test.getModuleVelocityMPS());
        
    }

    public void teleopInit() {
        swerve.writeOffsets();
        swerve.setBrake();
        // swerve.setCoast();
        swerve.readoffsets();
        swerve.updateOffsets();
        
    }

    public void disabledInit(){
        swerve.setCoast();
    }

    public void autonomousInit(){

    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void testExit() {

    }

}
