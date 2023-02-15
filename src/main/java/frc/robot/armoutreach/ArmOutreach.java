// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.control.motors.NKSpark;
import frc.robot.Constants;

public class ArmOutreach extends SubsystemBase {
  /** Creates a new ArmOutreach. */
  // private NKSpark outreach;
  private CANSparkMax outreach;
  private SparkMaxPIDController pid;
  private RelativeEncoder encoder;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  

  public ArmOutreach() {
    outreach = new CANSparkMax(Constants.kExtendMotor, MotorType.kBrushless); 
    pid = outreach.getPIDController();
    encoder = outreach.getEncoder();
    outreach.setIdleMode(IdleMode.kBrake);
    // outreach.setIdleMode(IdleMode.kCoast);
    encoder.setPosition(0); // resets the encoder

    outreach.restoreFactoryDefaults();

    kP = 0.1; 
    kI = 1e-4;
    kD = 1; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void forward(){
    outreach.set(-0.2);
    // System.out.println("INSIDE THE FORWARD");
  }
  public void stop(){
    outreach.stopMotor();
  }
  public void reverse(){
    outreach.set(0.1);
  }
  public void updateBoard(){
    SmartDashboard.putNumber("Arm Encoder",encoder.getPosition());
  }
}
