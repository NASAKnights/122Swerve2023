// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.control.motors.NKSpark;
import frc.robot.Constants;

public class ArmOutreach extends SubsystemBase {
  /** Creates a new ArmOutreach. */
  private NKSpark outreach;
  public ArmOutreach() {
    outreach = new NKSpark(Constants.kExtendMotor);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void forward(){
    outreach.set(0.1);
  }
  public void stop(){
    outreach.stopMotor();
  }
  public void reverse(){
    outreach.set(0.1);
  }
}
