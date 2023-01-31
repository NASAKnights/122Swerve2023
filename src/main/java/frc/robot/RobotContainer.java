package frc.robot;

import static frc.robot.Constants.kDriverPort;
import static frc.robot.Constants.kNavXPort;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.AutoCommand;
import frc.robot.drive.commands.DriveCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;
    private SwerveDrive swerve;

    public RobotContainer() {
        driver = new Joystick(kDriverPort);

        navx = new AHRS(kNavXPort);

    
        swerve = new SwerveDrive(navx);
        swerve.readoffsets();
        swerve.initDashboard();

        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new DriveCommand(driver, swerve));
    }

    private void configureButtonBindings() {
        new JoystickButton(driver, 1).onTrue(new InstantCommand(swerve::resetHeading));
        
    }

    public void periodic() {
        swerve.updateSmartDash();
        swerve.writeOffsets();
        swerve.readoffsets();
        // swerve.initDashboard();
        // SmartDashboard.putNumber("Module Velocity", test.getModuleVelocityMPS());
        
    }

    public void teleopInit() {
        swerve.writeOffsets();
        swerve.setBrake();
        swerve.readoffsets();
        swerve.updateOffsets();
    }

    public void disabledInit(){
        swerve.setCoast();
    }

    public CommandBase autonomousInit(){
        return new AutoCommand(swerve);
    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void testExit() {

    }

}
