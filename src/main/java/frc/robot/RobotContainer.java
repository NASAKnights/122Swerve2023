package frc.robot;

import static frc.robot.Constants.kNavXPort;

import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.HandOffSequence;
import frc.robot.armoutreach.commands.GoToHP;
import frc.robot.armoutreach.commands.GoToHigh;
import frc.robot.armoutreach.commands.GoToLow;
import frc.robot.armoutreach.commands.GoToMid;
import frc.robot.armoutreach.commands.LowerArm;
import frc.robot.armoutreach.commands.RetractFully;
import frc.robot.armoutreach.commands.StowInside;
import frc.robot.auto.SequentialCommands.AutoScoreHighBalance;
import frc.robot.auto.commands.AutoBalance;
import frc.robot.claw.Claw;
import frc.robot.claw.commands.CloseClaw;
import frc.robot.claw.commands.OpenClaw;
import frc.robot.colorSensor.ColorInterpreter;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.commands.DriveCommand;
import frc.robot.drive.commands.ToggleSlow;
import frc.robot.drive.commands.ToggleTurbo;
import frc.robot.intake.Intake;
import frc.robot.intake.StowIntakeSequence;
import frc.robot.intake.commands.EjectCone;
import frc.robot.intake.commands.EjectCube;
import frc.robot.intake.commands.IntakeCone;
import frc.robot.intake.commands.IntakeCube;
import frc.robot.intake.commands.LiftIntake;
import frc.robot.intake.commands.LowerIntake;
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

    private DigitalInput rotaryEncoderLoading = new DigitalInput(1);
    private DigitalInput rotaryEncoderBalance = new DigitalInput(2);
    private DigitalInput rotaryEncoderCable = new DigitalInput(4);
    private DigitalInput redBlueSwitch = new DigitalInput(5);
    private DigitalInput highMidSwitch = new DigitalInput(6);

    private String location = "unknown";
    private String alliance = "unknown";
    private String highLow = "unknown";

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
    }

    private void configureButtonBindings() {
        new JoystickButton(driver, 1).onTrue(new InstantCommand(swerve::resetHeading));
        
        // new JoystickButton(driver,4).onTrue(new DriveForwardTime(swerve, 2));
        new JoystickButton(driver, 7).onTrue(new OpenClaw(claw));
        new JoystickButton(driver,8).onTrue(new CloseClaw(claw));

        new JoystickButton(driver, 5).onFalse(new ToggleSlow(swerve))
                                                .onTrue(new ToggleTurbo(swerve));

        //------------Operator Buttons----------------------------------------------------------
        
        // new JoystickButton(operator, 1).whileTrue(new RepeatCommand (new ExtendToLength(arm)));
        // new JoystickButton(operator, 4).whileTrue(new RepeatCommand (new Retract(arm)));

        // new JoystickButton(operator, 7).whileTrue(new RepeatCommand(new LiftArm(arm)));
        // new JoystickButton(operator, 7).whileTrue(new RepeatCommand(new LiftToAngle(arm)));
        // new JoystickButton(operator, 8).whileTrue(new RepeatCommand(new LowerArm(arm)));
        // new JoystickButton(operator, 8).whileTrue(new RepeatCommand(new LowerToAngle(arm)));

        // new JoystickButton(operator, 1).whileTrue(new RepeatCommand(new GoInside(arm)));
        HandOffSequence handoff = new HandOffSequence(arm, intake, claw, indexer);
        Trigger op1 = new JoystickButton(operator, 1);
        op1.onTrue(handoff);
        Trigger op2 = new JoystickButton(operator, 2);
        op2.whileTrue(new RepeatCommand(new GoToLow(arm)));
        Trigger op3 = new JoystickButton(operator, 3);
        op3.whileTrue(new RepeatCommand(new GoToMid(arm)));
        Trigger op4 = new JoystickButton(operator, 4);
        op4.whileTrue(new RepeatCommand(new GoToHigh(arm)));
        Trigger op7 =  new JoystickButton(operator, 7);
        Trigger op8 = new JoystickButton(operator, 8);
        
        op7.whileTrue(new RepeatCommand(new IntakeCone(intake, arm)));
        op8.whileTrue(new RepeatCommand(new IntakeCube(intake, arm)));

        POVButton povDown = new POVButton(operator, 180);
        POVButton povLeft = new POVButton(operator, 270);
        POVButton povRight = new POVButton(operator, 90);
        povDown.onTrue(new StowIntakeSequence(arm, intake));
        povRight.whileTrue(new RepeatCommand(new EjectCube(intake)));
        povLeft.whileTrue(new RepeatCommand(new EjectCone(intake)));

        // new ConditionalCommand(new StowIntake(intake, arm), new InstantCommand(),
        //     () -> !operator.getRawButton(7) && !operator.getRawButton(8));
        // new JoystickButton(operator,5).whileTrue(new RepeatCommand(new LiftIntake(intake)));
        // new JoystickButton(operator,6).whileTrue(new RepeatCommand(new LowerIntake(intake)));
        new JoystickButton(operator, 5).whileTrue(new RepeatCommand(new StowInside(arm)));
        new JoystickButton(operator, 6).whileTrue(new RepeatCommand(new GoToHP(arm)));
        
        new JoystickButton(operator, 9).whileTrue(new RepeatCommand(new RetractFully(arm)));
        new JoystickButton(operator, 10).whileTrue(new RepeatCommand(new LowerArm(arm)));


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

        // SmartDashboard.putBoolean("Auto Side", rotaryEncoderLoading.get());
        // SmartDashboard.putBoolean("Red Blue Switch", redBlueSwitch.get());
        // SmartDashboard.putBoolean("Auto Balance", rotaryEncoderBalance.get());
        // SmartDashboard.putBoolean("AutoLowHigh", rotaryEncoderHighLow.get());
    }

    public void disabledPeriodic(){
        if(highMidSwitch.get()){
            highLow = "High";
        }
        else{
            highLow = "Mid";
        }

        if(redBlueSwitch.get()){
            alliance = "Blue";
        }
        else{
            alliance = "Red";
        }

        if(!rotaryEncoderLoading.get()){
            location = "Loading";
        }
        else if(rotaryEncoderCable.get()){
            location = "Cable";
        }

        if(!rotaryEncoderBalance.get()){
            location = "Balance";
        }

        // SmartDashboard.putBoolean("Loading", rotaryEncoderLoading.get());
        // SmartDashboard.putBoolean("Balance", rotaryEncoderBalance.get());
        // SmartDashboard.putBoolean("Cable", rotaryEncoderCable.get());

        SmartDashboard.putString("Final", "Auto" + highLow + location + alliance);
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
    
    public CommandBase getAutonomousCommand() {
        var group = PathPlanner.loadPathGroup("none", new PathConstraints(0, 0));

        if(highMidSwitch.get()){
            highLow = "High";
        }
        else{
            highLow = "Mid";
        }

        if(redBlueSwitch.get()){
            alliance = "Blue";
        }
        else{
            alliance = "Red";
        }
        
        if(!rotaryEncoderLoading.get()){
            location = "Loading";
        }
        else if(rotaryEncoderCable.get()){
            location = "Cable";
        }

        if(!rotaryEncoderBalance.get()){
            location = "Balance";
        }

        group = PathPlanner.loadPathGroup(
            "Auto" + highLow + location + alliance,
            new PathConstraints(3, 2));
        
        
        HashMap<String, Command> eventMap = new HashMap<>();
            eventMap.put("wait", new WaitCommand(2));
            eventMap.put("score", new OpenClaw(claw));
            eventMap.put("stow", new StowIntakeSequence(arm, intake));
        
            eventMap.put("armHigh", new RepeatCommand(new GoToHigh(arm, true)));
            eventMap.put("armMid", new RepeatCommand(new GoToMid(arm)));
            eventMap.put("stowArm", new RepeatCommand(new StowInside(arm)));
        
            eventMap.put("intakeCube", new RepeatCommand(new IntakeCube(intake, arm)));
            eventMap.put("stowIntake", new RepeatCommand(new StowIntake(intake, arm)));
        
            eventMap.put("AutoScoreHighBalance", new AutoScoreHighBalance(swerve, intake, arm, claw));
            eventMap.put("autoBalance", new SequentialCommandGroup(new AutoBalance(swerve)));

        SwerveAutoBuilder builder = new SwerveAutoBuilder(
            swerve::getPose,
            swerve::resetPose,
            new PIDConstants(9, 0, 0),
            new PIDConstants(3, 0, 0),
            swerve::drive,
            eventMap,
            swerve
        );

        return builder.fullAuto(group);
    }

    public CommandBase autonomousInit(){
        return new AutoScoreHighBalance(swerve, intake, arm, claw);
    }

    public void testInit() {

    }

    public void testPeriodic() {

    }

    public void testExit() {

    }

}
