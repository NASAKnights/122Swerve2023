package frc.robot;

import static frc.robot.Constants.kDriverPort;
import static frc.robot.Constants.kNavXPort;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.drive.ModuleTest;
import frc.robot.drive.SwerveDrive;
<<<<<<< HEAD
import frc.robot.drive.SwerveModule;
=======
import frc.robot.drive.PhotonVision.PhotonVision;
>>>>>>> b29c64c (Added subsystem, AprTag ID display on SmrtDshbrd)
import frc.robot.drive.commands.DriveCommand;
import frc.robot.drive.commands.DriveForwardTime;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.SetIntakeForward;
import frc.robot.intake.commands.SetIntakeReverse;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;
    private SwerveDrive swerve;
    private PhotonVision photon;

    private Intake intake;

    public RobotContainer() {
        driver = new Joystick(kDriverPort);

        navx = new AHRS(kNavXPort);

<<<<<<< HEAD
        intake = new Intake();

=======
        photon = new PhotonVision();
>>>>>>> b29c64c (Added subsystem, AprTag ID display on SmrtDshbrd)
        swerve = new SwerveDrive(navx);
        swerve.readoffsets();
        // swerve.updateSmartDash();
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
        new JoystickButton(driver,4).onTrue(new DriveForwardTime(swerve, 2));
        
    }

    public void periodic() {
        swerve.updateSmartDash();
        swerve.writeOffsets();
        swerve.readoffsets();
<<<<<<< HEAD
        // test.updateSmartDash();
        // swerve.updateOffsets();
        
=======
        photon.updateSmartDash();
        // swerve.initDashboard();
>>>>>>> b29c64c (Added subsystem, AprTag ID display on SmrtDshbrd)
        // SmartDashboard.putNumber("Module Velocity", test.getModuleVelocityMPS());
        
    }

    public void disabledPeriodic(){
        swerve.updateSmartDash();
        swerve.writeOffsets();
        swerve.readoffsets();
        
        swerve.updateOffsets();
    }

    public void teleopInit() {
        swerve.setBrake();
        swerve.writeOffsets();
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
