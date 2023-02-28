// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  TalonSRX intakeMotor;
  double power = 0.5;

  public Intake() {
    intakeMotor = new TalonSRX(1); //some motor

  }

  public void setIntake(){
    intakeMotor.set(ControlMode.PercentOutput, power);
  }

  public void setReverse(){
    intakeMotor.set(ControlMode.PercentOutput, -power);
  }

  public void stop(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}