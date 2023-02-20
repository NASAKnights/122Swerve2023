// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach;

import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.control.motors.NKSpark;
import frc.robot.Constants;

public class ArmOutreach extends SubsystemBase {
  /** Creates a new ArmOutreach. */
  // private NKSpark outreach;
  private CANSparkMax outreach;
  private SparkMaxPIDController pid;
  private RelativeEncoder extendEncoder;

  private CANSparkMax arm;
  private CANSparkMax armFollower;
  private SparkMaxPIDController pidArm;
  private SparkMaxAbsoluteEncoder pivotAngle;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  private Transform3d robot2ee;
  private Transform3d robot2rotary;
  private Transform3d rotary2linear;
  private Transform3d linear2ee;

  private double up = 0.5;
  private double down = -0.5;


  public ArmOutreach() {
    outreach = new CANSparkMax(Constants.ArmConstants.kExtendMotor, MotorType.kBrushless); 
    pid = outreach.getPIDController();
    extendEncoder = outreach.getEncoder();

    arm = new CANSparkMax(Constants.ArmConstants.kPivotMotor, MotorType.kBrushless);
    armFollower = new CANSparkMax(Constants.ArmConstants.kPivotMotorFollower, MotorType.kBrushless);
    pidArm = arm.getPIDController();
    pivotAngle = arm.getAbsoluteEncoder(Type.kDutyCycle);
    
    outreach.restoreFactoryDefaults();
    arm.restoreFactoryDefaults();
    resetEncoder();

    arm.setIdleMode(IdleMode.kBrake);
    outreach.setIdleMode(IdleMode.kBrake);
    pivotAngle.setZeroOffset(0.47);
    pivotAngle.setInverted(true);

    // pivotAngle.setPositionConversionFactor(2 * Math.PI /8192.0); // Sets Units of Encoder to radians

    outreach.setSmartCurrentLimit(40);
    arm.setSmartCurrentLimit(40);

    armFollower.follow(arm, true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setInitialPID(){
    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    pid.setIZone(kIz);
    pid.setFF(kFF);
    pid.setOutputRange(kMinOutput, kMaxOutput);

    pidArm.setP(kP);
    pidArm.setI(kI);
    pidArm.setD(kD);
    pidArm.setIZone(kIz);
    pidArm.setFF(kFF);
    pidArm.setOutputRange(kMinOutput, kMaxOutput);

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

  public void resetEncoder(){
    extendEncoder.setPosition(0.0);

  }

  public void forward(){
    outreach.set(-0.2);
    // System.out.println("INSIDE THE FORWARD");
  }
  public void stop(){
    outreach.stopMotor();
  }
  public void reverse(){
    outreach.set(0.2);
  }

  public void extendFully(){
    pid.setReference(Constants.ArmConstants.kExtensionRotations, CANSparkMax.ControlType.kPosition);
  }

  public void liftArm(){
    // pidArm.setReference(Constants.kArmUp, CANSparkMax.ControlType.kPosition);
    arm.set(0.1);
  }
  public void lowerArm(){
    arm.set(-0.1);
  }
  public void stopArm(){
    arm.stopMotor();
  }

  /**
   * Make the end effector go to an x/y position
   * <p>
   * Can be used to set x/y positions to pick/place various game elements
   * 
   * @param pos should contain the x y position that the arm needs to go to relative to the axle
   * of the pivot arm
   */
  public void gotoXY(Translation2d pos) {
    // Find the goal positions
    double pivotGoal = pos.getAngle().getRadians();
    double extendGoal = pos.getNorm(); // Maybe minus the arm length while retracted affects lines marked asdf
    
    // Check if goals are within reach since the pivot and extension can only go so far
    
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html
    // Send goals to lower level control loops (motor controllers)
    // A naive approach can apply a feedforward since we know how much torque we can expect the arm to be under
    // The feedforward will be proportional to the torque that the motor needs to exert to maintain it's position
    // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-intro.html#visualizing-feedforward
  }

  /**
   * @return Transform3d for robot2ee
   */
  public Transform3d getXY() {
    return robot2ee;
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
    Translation3d curPos = new Translation3d(extendEncoderToMeters(extendEncoder.getPosition()),0,0); // asdf
    linear2ee = new Transform3d(curPos, curRot);
  }

  /**
   * Converts the extend encoders value to meters
   * @param val the number of rotations that the motor has detected
   * @return the number of meters that the arm is extended
   */
  private double extendEncoderToMeters(double val) {
    double newVal = 0;

    // TODO: Write me
    
    return newVal;
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
    curPose.plus(rotary2linear.plus(linear2ee));

    robot2ee = curPose;
  }



  public void updateBoard(){
    SmartDashboard.putNumber("Extend Encoder",extendEncoder.getPosition());
    SmartDashboard.putNumber("Pivot Encoder Absolute", pivotAngle.getPosition());
    
  }
}
