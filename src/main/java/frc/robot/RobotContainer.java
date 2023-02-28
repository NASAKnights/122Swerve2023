package frc.robot;

import static frc.robot.Constants.kNavXPort;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.commands.ExtendToLength;
import frc.robot.armoutreach.commands.GoToHigh;
import frc.robot.armoutreach.commands.LiftArm;
import frc.robot.armoutreach.commands.LiftToAngle;
import frc.robot.armoutreach.commands.LowerArm;
import frc.robot.armoutreach.commands.LowerToAngle;
import frc.robot.armoutreach.commands.Retract;
import frc.robot.claw.Claw;
import frc.robot.claw.commands.CloseClaw;
import frc.robot.claw.commands.OpenClaw;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.colorSensor.ColorInterpreter;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.LiftIntake;
import frc.robot.intake.commands.LowerIntake;
import frc.robot.intake.commands.SetIntakeForward;
import frc.robot.intake.commands.SetIntakeReverse;

public class RobotContainer {

    private Joystick driver;
    private Joystick operator;

    private AHRS navx;
    private SwerveDrive swerve;

    private Intake intake;
    private ArmOutreach arm;

    private PneumaticHub pHub;
    private Claw claw;

    private ColorInterpreter indexer;

    public RobotContainer() {
        driver = new Joystick(Constants.kDriverPort);
        operator = new Joystick(Constants.kOperatorPort);

        navx = new AHRS(kNavXPort);

        intake = new Intake();

        claw = new Claw();
        arm = new ArmOutreach();

        indexer = new ColorInterpreter();

        pHub = new PneumaticHub(Constants.PneumaticConstants.kPneumaticHubModule); // 2
        pHub.enableCompressorAnalog(Constants.PneumaticConstants.kMinPressure, Constants.PneumaticConstants.kMaxPressure);
        swerve = new SwerveDrive(navx);
        
        swerve.readoffsets();
        swerve.initDashboard();
        swerve.updateSmartDash();
        // swerve.updateOffsets();

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
        
        // new JoystickButton(driver,4).onTrue(new DriveForwardTime(swerve, 2));
        new JoystickButton(driver, 7).onTrue(new OpenClaw(claw));
        new JoystickButton(driver,8).onTrue(new CloseClaw(claw));
        
        new JoystickButton(operator, 1).whileTrue(new RepeatCommand (new ExtendToLength(arm)));
        new JoystickButton(operator, 4).whileTrue(new RepeatCommand (new Retract(arm)));

        // new JoystickButton(operator, 7).whileTrue(new RepeatCommand(new LiftArm(arm)));
        new JoystickButton(operator, 7).whileTrue(new RepeatCommand(new LiftToAngle(arm)));
        // new JoystickButton(operator, 8).whileTrue(new RepeatCommand(new LowerArm(arm)));
        new JoystickButton(operator, 8).whileTrue(new RepeatCommand(new LowerToAngle(arm)));
        
        new JoystickButton(operator, 2).whileTrue(new RepeatCommand(new SetIntakeForward(intake)));
        new JoystickButton(operator, 3).whileTrue(new RepeatCommand(new SetIntakeReverse(intake)));
        new JoystickButton(operator,5).whileTrue(new RepeatCommand(new LiftIntake(intake)));
        new JoystickButton(operator,6).whileTrue(new RepeatCommand(new LowerIntake(intake)));

        new JoystickButton(operator, 9).whileTrue(new RepeatCommand(new GoToHigh(arm)));

        
    }

    public void periodic() {
        swerve.updateSmartDash();
        swerve.writeOffsets();
        swerve.readoffsets();
        SmartDashboard.putNumber("Pressure", pHub.getPressure(0));
        // test.updateSmartDash();
        arm.updateBoard();
        
        indexer.checkIndex();
        intake.updateBoard();
        
    }

    public void disabledPeriodic(){
        swerve.updateSmartDash();
        swerve.writeOffsets();
        swerve.readoffsets();
        
        swerve.updateOffsets();
        arm.updateBoard();
        intake.updateBoard();
    }

    public void teleopInit() {
        swerve.setBrake();
        swerve.writeOffsets();
        // swerve.setCoast();
        swerve.readoffsets();
        swerve.updateOffsets();
        arm.updateBoard();
       
        
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
