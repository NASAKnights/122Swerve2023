package frc.robot;

import static frc.robot.Constants.kDriverPort;
import static frc.robot.Constants.kNavXPort;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.DriveCommand;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;
    private SwerveDrive swerve;

    public RobotContainer() {
        driver = new Joystick(kDriverPort);

        navx = new AHRS(kNavXPort);

        swerve = new SwerveDrive(navx);

        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new DriveCommand(driver, swerve));
    }

    private void configureButtonBindings() {
        new JoystickButton(driver, 1).onTrue(new InstantCommand(swerve::resetHeading));
        

        // new JoystickButton(driver, 2).whileHeld(new VelocityTest(test, 0.5));
        // new JoystickButton(driver, 3).whileHeld(new VoltageTest(test, 1));
        // SmartDashboard.putNumber("volts", 0);
    }

    public void periodic() {
        swerve.updateSmartDash();
        // SmartDashboard.putNumber("Module Velocity", test.getModuleVelocityMPS());
    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void testExit() {

    }

}
