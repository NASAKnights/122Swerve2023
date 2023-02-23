// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // TalonSRX intakeMotor;
  VictorSP intakeMotor;
  CANSparkMax intakeLiftMotor;

  RelativeEncoder intakeLiftEncoder;
  SparkMaxPIDController intakeLiftPID; 

  public Intake() {
    intakeMotor = new VictorSP(Constants.IntakeConstants.kIntakeMotor); // PWM channels
    intakeLiftMotor = new CANSparkMax(Constants.IntakeConstants.kLiftMotor, MotorType.kBrushless);
    intakeLiftMotor.setIdleMode(IdleMode.kBrake);

    intakeLiftEncoder = intakeLiftMotor.getEncoder();
    intakeLiftPID = intakeLiftMotor.getPIDController();
  
  }

  public void setIntake(){
    intakeMotor.set(0.2);
  }

  public void setReverse(){
    intakeMotor.set(-0.2);
  }

  public void setIntakePivot(double degrees){
    

  }

  public void stopIntake(){
    intakeMotor.stopMotor();
  }

  public void liftIntake(){
    intakeLiftMotor.set(0.1);

  }
  public void lowerIntake(){
    intakeLiftMotor.set(-0.1);
  }

  public void stopIntakeLift(){
    intakeLiftMotor.stopMotor();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
