// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.control.motors.NKSpark;
import frc.robot.Constants;

public class ArmOutreach extends SubsystemBase {
  /** Creates a new ArmOutreach. */
  // private NKSpark outreach;
  private CANSparkMax outreach;
  private SparkMaxPIDController extendPID;
  private RelativeEncoder extendEncoder;

  private CANSparkMax arm;
  private CANSparkMax armFollower;
  private SparkMaxPIDController pivotPID;
  private SparkMaxAbsoluteEncoder pivotAngle; // through bore encoder
  private RelativeEncoder pivotAngleQuad; // integrated encoder


  public double kPextend, kIextend, kDextend, kIzextend, kFFextend, kMaxOutputextend, kMinOutputextend;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


  private Transform3d robot2ee;
  // private Transform3d robot2rotary;
  private Transform3d rotary2linear;
  private Transform3d linear2ee;

  public ArmOutreach() {

    outreach = new CANSparkMax(Constants.ArmConstants.kExtendMotor, MotorType.kBrushless); 
    arm = new CANSparkMax(Constants.ArmConstants.kPivotMotor, MotorType.kBrushless);
    armFollower = new CANSparkMax(Constants.ArmConstants.kPivotMotorFollower, MotorType.kBrushless);
    
    outreach.restoreFactoryDefaults();
    arm.restoreFactoryDefaults();
    
    extendPID = outreach.getPIDController();
    extendEncoder = outreach.getEncoder();

    pivotPID = arm.getPIDController();
    pivotAngle = arm.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    // pivotAngleQuad = arm.getEncoder(SparkMaxRelativeEncoder.Type.kQuadrature, 8192);
    pivotAngleQuad = arm.getEncoder();

    pivotPID.setFeedbackDevice(pivotAngleQuad);

    pivotAngle.setPositionConversionFactor(2 * Math.PI); // change from rotations to radians
    pivotAngleQuad.setPositionConversionFactor(2 * Math.PI); // change from rotations to radians

    resetExtensionEncoder();
    setInitialPID();
    
    outreach.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    outreach.setIdleMode(IdleMode.kBrake);

    pivotAngle.setZeroOffset(0.474); // 268.23
    pivotAngle.setInverted(true);

    // extendEncoder.setPositionConversionFactor((Constants.ArmConstants.kExtensionLength /Constants.ArmConstants.kExtensionRotations));
    extendEncoder.setPositionConversionFactor(Constants.ArmConstants.kExtensionRatio); // change from rotations to meters (depends on motor)
    outreach.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);
    arm.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);

    arm.setClosedLoopRampRate(Constants.ArmConstants.kPivotClosedLoopRamp);

    resetPivotToAbsolute();

    armFollower.follow(arm, true);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    checkRetracted();
  }

  public void initEncoders(){

  }

  public void setInitialPID(){
    kPextend = 1.5; 
    kIextend = 5e-4;
    kDextend = 0.01; 
    kIzextend = 0; 
    kFFextend = 0; 
    kMaxOutputextend = 1; 
    kMinOutputextend = -1;

    kP = 0.9; 
    kI = 1e-4;
    kD = 0.01; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 0.1; 
    kMinOutput = -0.1;

    extendPID.setP(kPextend);
    extendPID.setI(kIextend);
    extendPID.setD(kDextend);
    extendPID.setIZone(kIzextend);
    extendPID.setFF(kFFextend);
    extendPID.setOutputRange(kMinOutputextend, kMaxOutputextend);

    pivotPID.setP(kP);
    pivotPID.setI(kI);
    pivotPID.setD(kD);
    pivotPID.setIZone(kIz);
    pivotPID.setFF(kFF);
    pivotPID.setOutputRange(kMinOutput, kMaxOutput);

    pivotPID.setPositionPIDWrappingEnabled(true); //see what happens

  }

  public void setArmFF(){
    // Will change as arm moves

    // torque = r_1 * w_1 * sin(theta_1) + (d_2 + l_2 + o_2)* w_2 * sin(theta_1);
    // theta_1 = angle of arm
    // r_1 = distance to cog of joint 1
    // w_1 = weight of joint 1
    // d_2 = distance to start of linear motion for joint 2
    // l_2 = extention of joint 2
    // o_2 = offset from end of joint 2 to cog of joint 2
  }

  public void checkRetracted(){
    if (outreach.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed()){
      resetExtensionEncoder(); // resets the extension encoder when the limit switch is pressed
    }
  }

  public void resetExtensionEncoder(){
    extendEncoder.setPosition(0.0);
  }

  public void resetPivotToAbsolute(){
    pivotAngleQuad.setPosition(pivotAngle.getPosition());
  }

  public void forward(){
    outreach.set(0.2);
  }
  public void stop(){
    outreach.stopMotor();
  }
  public void reverse(){
    outreach.set(-0.2);
  }

  public void extendFully(){
    extendPID.setReference(0.375, CANSparkMax.ControlType.kPosition);
  }
  public void retractToZero(){
    extendPID.setReference(0.0, CANSparkMax.ControlType.kPosition);
  }

  public void liftArm(){
    // pivotPID.setReference(Constants.kArmUp, CANSparkMax.ControlType.kPosition);
    arm.set(0.1);
  }
  public void lowerArm(){
    arm.set(-0.1);
  }
  public void stopArm(){
    arm.stopMotor();
  }

  public void liftArmtoAngle(){
    pivotPID.setReference(0.1 * Math.PI, ControlType.kPosition);
    
  }
  public void lowerArmtoAngle(){
    pivotPID.setReference(-0.25 * Math.PI, ControlType.kPosition);

  }

  /**
   * Make the end effector go to an x/y position
   * <p>
   * Can be used to set x/y positions to pick/place various game elements
   * 
   * @param pos should contain the x y position that the arm needs to go to relative to the axle
   * of the pivot arm
   */
  public boolean gotoXY(Translation2d pos) {
    // Find the goal positions
    double pivotGoal = pos.getAngle().getRadians();
    double extendGoal = pos.getNorm() - Constants.ArmConstants.kExtentionRetractedLength; // Maybe minus the arm length while retracted affects lines marked asdf
    
    // Check if goals are within reach since the pivot and extension can only go so far
    if(pivotGoal > Constants.ArmConstants.kPivotMaxRotation || 
    extendGoal > Constants.ArmConstants.kExtensionLength ||
    pivotGoal < Constants.ArmConstants.kPivotMinRotation ||
    extendGoal < Constants.ArmConstants.kExtentionMinLength){
      return false;
    }
    
    // Send goals to lower level control loops (motor controllers)
    extendPID.setReference(extendGoal, ControlType.kPosition);
    pivotPID.setReference(pivotGoal, ControlType.kPosition);

    // A naive approach can apply a feedforward since we know how much torque we can expect the arm to be under
    // The feedforward will be proportional to the torque that the motor needs to exert to maintain it's position
    return true;
  }

  /**
   * @return Transform3d for robot2ee
   */
  public Translation2d getXY() {
    double x = robot2ee.getX();
    double y = robot2ee.getY();
    Translation2d val = new Translation2d(x, y);
    return val;
  }

  /**
   * Updates the locally stored Pose of the rotary2linear Translation3d object
   * 
   */
  private void updatePivotPose() {
    // Updates the Pivot pose with the new angle from the encoder
    Rotation3d curRot = new Rotation3d(0, 0, pivotAngle.getPosition());
    Translation3d curPos = new Translation3d(0, 0, 0);
    rotary2linear = new Transform3d(curPos, curRot);
  }

  /**
   * Updates the locally stored Pose of the linear2ee Translation3d object
   */
  private void updateExtendPose() {
    // Updates the Extend Pose with the new position from the encoder
    Rotation3d curRot = new Rotation3d();
    Translation3d curPos = new Translation3d(
      extendEncoder.getPosition() + Constants.ArmConstants.kExtentionRetractedLength,
      0,
      0); // asdf
    linear2ee = new Transform3d(curPos, curRot);
  }


  /**
   * Updates all transforms in the arm and then use forward kinematics to update the End Effector (ee) Position
   * <p>
   * robot2ee_x = (arm_len + arm_ext)*cos(arm_angle) 
   * <p>
   * robot2ee_y = (arm_len + arm_ext)*sin(arm_angle)
   */
  private void updateEEPos() {
    // 
    updateExtendPose();
    updatePivotPose();
    Transform3d curPose = new Transform3d();
    robot2ee = curPose.plus(rotary2linear.plus(linear2ee));

    // robot2ee = curPose;
  }



  public void updateBoard(){
    SmartDashboard.putNumber("Extend Encoder",extendEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Encoder Absolute", pivotAngle.getPosition());
    SmartDashboard.putNumber("Pivot Encoder Relative", pivotAngleQuad.getPosition());
    updateEEPos();
    SmartDashboard.putNumber("Arm X Pos", robot2ee.getX());
    SmartDashboard.putNumber("Arm Y Pos", robot2ee.getY());
    // SmartDashboard.putBoolean("Limit F switch", limitSwitchPressed());

  }
}
