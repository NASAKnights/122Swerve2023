package frc.robot;

import static frc.robot.Constants.kDriverPort;
import static frc.robot.Constants.kNavXPort;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.ModuleTest;
import frc.robot.drive.SwerveDrive;
<<<<<<< HEAD
import frc.robot.drive.SwerveModule;
=======
import frc.robot.drive.PhotonVision.PhotonVision;
import frc.robot.drive.commands.AutoCommand;
>>>>>>> 22cce51749046aaab5f9e898b29a8aed8bd9ecf7
import frc.robot.drive.commands.DriveCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.intake.Intake;
import frc.robot.intake.commands.SetIntakeForward;
import frc.robot.intake.commands.SetIntakeReverse;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;

    private PhotonVision photon;
    private SwerveDrive swerve;

    private Intake intake;

    public RobotContainer() {
        driver = new Joystick(kDriverPort);

        navx = new AHRS(kNavXPort);

        intake = new Intake();

        photon = new PhotonVision();
        swerve = new SwerveDrive(navx);
        swerve.readoffsets();
        swerve.initDashboard();

        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(new DriveCommand(driver, swerve));
        // swerve.setDefaultCommand(new VelocityTest(swerve, 0.0));
        // swerve.setDefaultCommand(new VoltageTest(swerve, 0));
        // test.setDefaultCommand(new ModuleTestCommand(driver, test));
    }

    private void configureButtonBindings() {
        new JoystickButton(driver, 1).onTrue(new InstantCommand(swerve::resetHeading));
        new JoystickButton(driver, 2).whileTrue(new RepeatCommand(new SetIntakeForward(intake)));
        new JoystickButton(driver, 3).whileTrue(new RepeatCommand(new SetIntakeReverse(intake)));
        
    }

    public void periodic() {
        swerve.updateSmartDash();
<<<<<<< HEAD
        
=======
        swerve.writeOffsets();
        swerve.readoffsets();
        photon.updateSmartDash();
        // swerve.initDashboard();
>>>>>>> 22cce51749046aaab5f9e898b29a8aed8bd9ecf7
        // SmartDashboard.putNumber("Module Velocity", test.getModuleVelocityMPS());
        
    }

    public void teleopInit() {
        swerve.setBrake();
       
        
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
