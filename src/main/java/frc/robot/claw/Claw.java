// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.claw;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.control.pneumatics.NKDoubleSolenoid;
import frc.robot.Constants;

public class Claw extends SubsystemBase {
  /** Creates a new Claw. */
  // private PneumaticHub pHub;
  private NKDoubleSolenoid clawSolenoid;

  public Claw() {

    // this.pHub = pHub;
    this.clawSolenoid = new NKDoubleSolenoid(PneumaticsModuleType.REVPH, Constants.PneumaticConstants.kSolenoidForward, Constants.PneumaticConstants.kSolenoidReverse);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void checkPressure(){
    // double pressure = pHub.getPressure(Constants.PneumaticConstants.kAnalogPressureChannel);
    // SmartDashboard.putNumber("Pressure", pressure);

  }

  public void forward(){
    // clawSolenoid.set(Value.kForward);
  }

  public void reverse(){
    // clawSolenoid.set(Value.kReverse);
  }

  public void off(){
    // clawSolenoid.set(Value.kOff);
  }

}
