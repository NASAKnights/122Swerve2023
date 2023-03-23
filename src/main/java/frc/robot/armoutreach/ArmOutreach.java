// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
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

  private REVLibError sticky = REVLibError.kError;


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
    pivotAngle.setZeroOffset(0.6372 - 0.0155); // 268.23

    pivotAngle.setPositionConversionFactor(2 * Math.PI); // change from rotations to radians
    // pivotAngleQuad.setPositionConversionFactor(2 * Math.PI); // change from rotations to radians
    // (4.5/1)
    pivotAngleQuad.setPositionConversionFactor((1.0 / 45.0 /4.5) * 2 * Math.PI);

    resetExtensionEncoder();
    setInitialPID();
    
    outreach.setInverted(true);
    arm.setIdleMode(IdleMode.kBrake);
    armFollower.setIdleMode(IdleMode.kBrake);
    outreach.setIdleMode(IdleMode.kBrake);

    pivotAngle.setInverted(true);

    // extendEncoder.setPositionConversionFactor((Constants.ArmConstants.kExtensionLength /Constants.ArmConstants.kExtensionRotations));
    extendEncoder.setPositionConversionFactor(Constants.ArmConstants.kExtensionRatio); // change from rotations to meters (depends on motor)
    outreach.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);
    arm.setSmartCurrentLimit(Constants.ArmConstants.kCurrentLimit);

    arm.setClosedLoopRampRate(Constants.ArmConstants.kPivotClosedLoopRamp);

    // pivotAngleQuad.setPosition(0.0);

    armFollower.follow(arm, true);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // checkRetracted();
  }

  public void initEncoders(){

  }

  public void setInitialPID(){
    kPextend = 1.5;
    kIextend = 5e-4;
    kDextend = 1e-3;
    kIzextend = 0.25;
    kFFextend = 0.35; 
    kMaxOutputextend = 1.0; 
    kMinOutputextend = -1.0;

    kP = 1.0; 
    kI = 1e-4;
    kD = 0.01; 
    kIz = 0; 
    kFF = -0.2; 
    kMaxOutput = 0.9; 
    kMinOutput = -0.9;

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
    if (isRetracted()){
      resetExtensionEncoder(); // resets the extension encoder when the limit switch is pressed
    }
  }

  public void resetExtensionEncoder(){
    extendEncoder.setPosition(0.0);
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
    outreach.set(-0.4);
  }

  public double getExtendLength(){
    return extendEncoder.getPosition();
  }

  public boolean isRetracted(){
    return outreach.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
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
    pivotPID.setReference(2.15 * Math.PI, ControlType.kPosition);
    
  }
  public void lowerArmtoAngle(){
    pivotPID.setReference(1.5 * Math.PI, ControlType.kPosition);

  }

  public double getArmAngle(){
    return pivotAngle.getPosition();
  }

  public void setArmToAngle(double angle){
    pivotPID.setReference(angle, ControlType.kPosition);
  }

  public void cycleAbsolute(){
    if (sticky != REVLibError.kOk){
      double angle = pivotAngle.getPosition();
      if (angle < Math.PI)
      {
        angle += (Math.PI*2);
      }
      sticky = pivotAngleQuad.setPosition(angle);
    }
  }

  /**
   * Make the end effector go to an x/y position
   * <p>
   * Can be used to set x/y positions to pick/place various game elements
   * Pivots the arm before extension/retraction
   * 
   * @param pos should contain the x y position that the arm needs to go to relative to the axle
   * of the pivot arm
   */
  public boolean gotoXY(Translation2d pos) {
    // Find the goal positions
    double pivotGoal = MathUtil.angleModulus(pos.getAngle().getRadians()) + (2*Math.PI);
    double extendGoal = pos.getNorm() - Constants.ArmConstants.kExtensionRetractedLength; // Maybe minus the arm length while retracted affects lines marked asdf
    SmartDashboard.putNumber("PivotGoal", pivotGoal);
    SmartDashboard.putNumber("extendGoal", extendGoal);

    if (extendGoal < 0) {
      extendGoal = 0.0;
    }
    
    // Check if goals are within reach since the pivot and extension can only go so far
    if(pivotGoal > Constants.ArmConstants.kPivotMaxRotation || 
    extendGoal > Constants.ArmConstants.kExtensionLength ||
    pivotGoal < Constants.ArmConstants.kPivotMinRotation ||
    extendGoal < Constants.ArmConstants.kExtentionMinLength){
      System.out.println("FFFFFFFFFFFFFFFFFFFFFFFFFFFF");
      return false;
    }
    
    // Send goals to lower level control loops (motor controllers)
    pivotPID.setReference(pivotGoal, ControlType.kPosition);
    if (Math.abs(pivotAngleQuad.getPosition() - pivotGoal) < 0.075 * Math.PI){
      extendPID.setReference(extendGoal, ControlType.kPosition);
    }

    // A naive approach can apply a feedforward since we know how much torque we can expect the arm to be under
    // The feedforward will be proportional to the torque that the motor needs to exert to maintain it's position
    return true;
  }

  //----------------------USE ONLY FOR STOWING CONE----------------------------

  /**
   * Make the end effector go to an x/y position
   * <p>
   * Can be used to set x/y positions to pick/place various game elements
   * Extends/retracts the arm before pivoting
   * 
   * @param pos should contain the x y position that the arm needs to go to relative to the axle
   * of the pivot arm
   */
  public boolean gotoXYextend(Translation2d pos) {
    // Find the goal positions
    double pivotGoal = MathUtil.angleModulus(pos.getAngle().getRadians()) + (2*Math.PI);
    double extendGoal = pos.getNorm() - Constants.ArmConstants.kExtensionRetractedLength; // Maybe minus the arm length while retracted affects lines marked asdf
    SmartDashboard.putNumber("PivotGoal", pivotGoal);
    SmartDashboard.putNumber("extendGoal", extendGoal);

    if (extendGoal < 0) {
      extendGoal = 0.0;
    }
    
    // Check if goals are within reach since the pivot and extension can only go so far
    if(pivotGoal > Constants.ArmConstants.kPivotMaxRotation || 
    extendGoal > Constants.ArmConstants.kExtensionLength ||
    pivotGoal < Constants.ArmConstants.kPivotMinRotation ||
    extendGoal < Constants.ArmConstants.kExtentionMinLength){
      System.out.println("FFFFFFFFFFFFFFFFFFFFFFFFFFFF");
      return false;
    }
    
    // Send goals to lower level control loops (motor controllers)
    extendPID.setReference(extendGoal, ControlType.kPosition);    
    if (Math.abs(extendEncoder.getPosition() - extendGoal) < 0.1){
      pivotPID.setReference(pivotGoal, ControlType.kPosition);
    }

    // A naive approach can apply a feedforward since we know how much torque we can expect the arm to be under
    // The feedforward will be proportional to the torque that the motor needs to exert to maintain it's position
    return true;
  }



  //---------------------------------------------------------------------------


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
      extendEncoder.getPosition() + Constants.ArmConstants.kExtensionRetractedLength,
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
    // SmartDashboard.putNumber("Pivot Encoder Absolute", pivotAngle.getPosition());
    SmartDashboard.putNumber("Pivot Encoder Relative", pivotAngleQuad.getPosition());
    SmartDashboard.putBoolean("Extend Limit",isRetracted());
    updateEEPos();
    SmartDashboard.putNumber("Arm X Pos", robot2ee.getX());
    SmartDashboard.putNumber("Arm Y Pos", robot2ee.getY());
    
    // SmartDashboard.putBoolean("Limit F switch", limitSwitchPressed());

  }
}
