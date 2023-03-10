package frc.robot;

import static frc.robot.Constants.kNavXPort;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.HandOffSequence;
import frc.robot.armoutreach.commands.ExtendToLength;
import frc.robot.armoutreach.commands.GoInside;
import frc.robot.armoutreach.commands.GoToHP;
import frc.robot.armoutreach.commands.GoToHigh;
import frc.robot.armoutreach.commands.GoToLow;
import frc.robot.armoutreach.commands.GoToMid;
import frc.robot.armoutreach.commands.HandOff;
import frc.robot.armoutreach.commands.LiftArm;
import frc.robot.armoutreach.commands.LiftToAngle;
import frc.robot.armoutreach.commands.LowerArm;
import frc.robot.armoutreach.commands.LowerToAngle;
import frc.robot.armoutreach.commands.Retract;
import frc.robot.armoutreach.commands.StowInside;
import frc.robot.auto.SequentialCommands.AutoOutOfCommunity;
import frc.robot.auto.SequentialCommands.AutoScoreLow;
import frc.robot.claw.Claw;
import frc.robot.claw.commands.CloseClaw;
import frc.robot.claw.commands.OpenClaw;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.drive.commands.ToggleTurbo;
import frc.robot.colorSensor.ColorInterpreter;
import frc.robot.drive.commands.DriveForwardTime;
import frc.robot.drive.commands.ToggleSlow;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.intake.Intake;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.LiftIntake;
import frc.robot.intake.commands.LowerIntake;
import frc.robot.intake.commands.SetIntakeForward;
import frc.robot.intake.commands.SetIntakeReverse;
import frc.robot.intake.commands.SetIntaketoAngle;
import frc.robot.intake.commands.StowIntake;

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

    // private ShuffleboardTab swerveModuleInfo = Shuffleboard.getTab("Swerve Modules");

    public RobotContainer() {
        driver = new Joystick(Constants.kDriverPort);
        operator = new Joystick(Constants.kOperatorPort);

        navx = new AHRS(kNavXPort);

        intake = new Intake();

        claw = new Claw();
        arm = new ArmOutreach();

        indexer = new ColorInterpreter();
        CameraServer.startAutomaticCapture();

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
        intake.setDefaultCommand(new StowIntake(intake));
    }

    private void configureButtonBindings() {
        new JoystickButton(driver, 1).onTrue(new InstantCommand(swerve::resetHeading));
        
        // new JoystickButton(driver,4).onTrue(new DriveForwardTime(swerve, 2));
        new JoystickButton(driver, 7).onTrue(new OpenClaw(claw));
        new JoystickButton(driver,8).onTrue(new CloseClaw(claw));

        new JoystickButton(driver, 5).onTrue(new ToggleSlow(swerve))
                                                .onFalse(new ToggleTurbo(swerve));

        //------------Operator Buttons----------------------------------------------------------
        
        // new JoystickButton(operator, 1).whileTrue(new RepeatCommand (new ExtendToLength(arm)));
        // new JoystickButton(operator, 4).whileTrue(new RepeatCommand (new Retract(arm)));

        // new JoystickButton(operator, 7).whileTrue(new RepeatCommand(new LiftArm(arm)));
        // new JoystickButton(operator, 7).whileTrue(new RepeatCommand(new LiftToAngle(arm)));
        // new JoystickButton(operator, 8).whileTrue(new RepeatCommand(new LowerArm(arm)));
        // new JoystickButton(operator, 8).whileTrue(new RepeatCommand(new LowerToAngle(arm)));

        // new JoystickButton(operator, 1).whileTrue(new RepeatCommand(new GoInside(arm)));
        new JoystickButton(operator, 1).onTrue(new HandOffSequence(arm, intake, claw, indexer));

        new JoystickButton(operator, 2).whileTrue(new RepeatCommand(new GoToLow(arm)));
        new JoystickButton(operator, 3).whileTrue(new RepeatCommand(new GoToMid(arm)));
        new JoystickButton(operator, 4).whileTrue(new RepeatCommand(new GoToHigh(arm)));
        
        new JoystickButton(operator, 7).whileTrue(new RepeatCommand(new IntakeCone(intake)));
        new JoystickButton(operator, 8).whileTrue(new RepeatCommand(new IntakeCube(intake)));
        // new JoystickButton(operator,5).whileTrue(new RepeatCommand(new LiftIntake(intake)));
        // new JoystickButton(operator,6).whileTrue(new RepeatCommand(new LowerIntake(intake)));
        new JoystickButton(operator, 5).whileTrue(new RepeatCommand(new StowInside(arm)));
        // new JoystickButton(operator, 6).whileTrue(new RepeatCommand(new SetIntaketoAngle(intake, Math.PI * 1.0)));
        new JoystickButton(operator, 6).whileTrue(new RepeatCommand(new GoToHP(arm)));

        BooleanEvent liftaxis = operator.axisGreaterThan(0, 0.15, new EventLoop());
        BooleanEvent loweraxis = operator.axisLessThan(0, -0.15, new EventLoop());

        new Trigger(liftaxis).whileTrue(new RepeatCommand(new LiftIntake(intake)));
        new Trigger(loweraxis).whileTrue(new RepeatCommand(new LowerIntake(intake)));

        
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
        arm.cycleAbsolute();
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

    public CommandBase autonomousInit(){
        // return new AutoSequencer(swerve);
        // arm.resetPivotToAbsolute();

        // return new AutoOutOfCommunity(swerve);
        return new AutoScoreLow(swerve, intake, arm);
             

    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void testExit() {

    }

}
