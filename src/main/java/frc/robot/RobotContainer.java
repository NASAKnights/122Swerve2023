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
<<<<<<< HEAD
>>>>>>> b29c64c (Added subsystem, AprTag ID display on SmrtDshbrd)
=======
import frc.robot.drive.commands.AutoCommand;
>>>>>>> c6bb8a7 (autocommand refs)
import frc.robot.drive.PhotonVision.PhotonVision;
import frc.robot.drive.commands.DriveCommand;
<<<<<<< HEAD
import frc.robot.drive.commands.DriveForwardTime;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.SetIntakeForward;
import frc.robot.intake.commands.SetIntakeReverse;
=======
import edu.wpi.first.wpilibj2.command.CommandBase;
>>>>>>> 0a03bcb (theived from autoDriveCommand)
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;


    private PhotonVision photon;
    private SwerveDrive swerve;

    private Intake intake;
    private PhotonVision photon;
    private SwerveDrive swerve;

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
