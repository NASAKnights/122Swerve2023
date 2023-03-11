// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.control.motors.NKTalonSRX;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  // TalonSRX intakeMotor;
  TalonSRX intakeMotor;
  CANSparkMax intakeLiftMotor;

  RelativeEncoder intakeLiftEncoder;
  SparkMaxPIDController intakeLiftPID; 

  private DigitalInput limitSwitch;

  // private double stowedPosition = 3.14;
  private double stowedPosition = 0.0;
  private double acceptableError = 0.01;

  public Intake() {
    intakeMotor = new TalonSRX(Constants.IntakeConstants.kIntakeMotor);
    intakeLiftMotor = new CANSparkMax(Constants.IntakeConstants.kLiftMotor, MotorType.kBrushless);
    intakeLiftMotor.setIdleMode(IdleMode.kBrake);
    intakeLiftMotor.setInverted(true);

    intakeMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 20.0, 25.0, 0.0));
    intakeLiftEncoder = intakeLiftMotor.getEncoder();
    intakeLiftPID = intakeLiftMotor.getPIDController();
    limitSwitch = new DigitalInput(0);

    intakeLiftPID.setP(0.3);
    intakeLiftPID.setI(1e-3);
    intakeLiftPID.setD(1);
    intakeLiftPID.setFF(0.0);
    intakeLiftPID.setIZone(0.5);
    intakeLiftPID.setOutputRange(-0.5, 0.5);

    intakeLiftEncoder.setPositionConversionFactor((1.0/45.0) * 2*Math.PI); // radian
    intakeLiftEncoder.setVelocityConversionFactor((1.0/45.0) * 2*Math.PI); // radian/s
    
    resetPivotEncoder();

  }

  public void runIntakeForward(){
    // takes in cone, outputs cube
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.9);
  }
  public void runIntakeReverse(){
    // outputs cone, takes in cube
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -0.6);
  }

  public void setIntake(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.6);
  }

  public void handOffCube() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.1);
  }

  public void setReverse(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -1);
  }

  public boolean isIntakeInside(){
    return !limitSwitch.get();
  }

  public void resetPivotEncoder(){
    intakeLiftEncoder.setPosition(0.0);
  }

  public double getAngle(){
    return intakeLiftEncoder.getPosition();
  }

  public void setIntakePivot(double angle){
    intakeLiftPID.setReference(angle, ControlType.kPosition);
  }

  public void stowIntake(){
    if(Math.abs(intakeLiftEncoder.getPosition() - stowedPosition) > acceptableError)
    {
      lowerIntake();
    }
  }

  public void stopIntake(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0);
  }

  public void liftIntake(){
    intakeLiftMotor.set(0.3);

  }
  public void lowerIntake(){
    intakeLiftMotor.set(-0.15);
  }

  public void intakeCube(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -0.6);
  }
  public void outputCube(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.6);
  }
  public void intakeCone(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.9);
  }
  public void outputCone(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, -0.9);
  }

  public void stopIntakeLift(){
    intakeLiftMotor.stopMotor();
  }

  public void updateBoard(){
    SmartDashboard.putNumber("Intake Position", intakeLiftEncoder.getPosition());
    SmartDashboard.putBoolean("Intake Limit", isIntakeInside());
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isIntakeInside()){
      resetPivotEncoder();
    }
  }
}