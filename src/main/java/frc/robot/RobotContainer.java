package frc.robot;

import static frc.robot.Constants.kDriverPort;
import static frc.robot.Constants.kNavXPort;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.commands.Extend;
import frc.robot.armoutreach.commands.Retract;
import frc.robot.claw.Claw;
import frc.robot.claw.commands.CloseClaw;
import frc.robot.claw.commands.OpenClaw;
import frc.robot.drive.ModuleTest;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.drive.commands.DriveForwardTime;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.SetIntakeForward;
import frc.robot.intake.commands.SetIntakeReverse;

public class RobotContainer {

    private Joystick driver;

    private AHRS navx;
    private SwerveDrive swerve;

    private Intake intake;
    private ArmOutreach arm;

    private PneumaticHub pHub;
    private Claw claw;

    public RobotContainer() {
        driver = new Joystick(kDriverPort);

        navx = new AHRS(kNavXPort);

        intake = new Intake();

        claw = new Claw();
        arm = new ArmOutreach();

        pHub = new PneumaticHub(2);
        pHub.enableCompressorAnalog(Constants.PneumaticConstants.kMinPressure, Constants.PneumaticConstants.kMaxPressure);
        swerve = new SwerveDrive(navx);
        swerve.readoffsets();

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
        // new JoystickButton(driver, 2).whileTrue(new RepeatCommand(new SetIntakeForward(intake)));
        // new JoystickButton(driver, 3).whileTrue(new RepeatCommand(new SetIntakeReverse(intake)));
        // new JoystickButton(driver,4).onTrue(new DriveForwardTime(swerve, 2));
        new JoystickButton(driver, 2).onTrue(new OpenClaw(claw));
        new JoystickButton(driver, 3).onTrue(new CloseClaw(claw));
        new JoystickButton(driver, 5).whileTrue(new RepeatCommand (new Extend(arm)));
        new JoystickButton(driver, 6).whileTrue(new RepeatCommand (new Retract(arm)));

        
    }

    public void periodic() {
        swerve.updateSmartDash();
        swerve.writeOffsets();
        swerve.readoffsets();
        SmartDashboard.putNumber("Pressure", pHub.getPressure(0));
        // test.updateSmartDash();
        
        
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
