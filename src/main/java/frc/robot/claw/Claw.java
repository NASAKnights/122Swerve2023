// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.claw;

import com.ctre.phoenix.led.CANdle.VBatOutputMode;

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
  // private NKDoubleSolenoid rightClawSolenoid;
  // private NKDoubleSolenoid leftClawSolenoid;
  private NKDoubleSolenoid clawSolenoid;

  public Claw() {

    // this.pHub = pHub;
    this.clawSolenoid = new NKDoubleSolenoid(2,PneumaticsModuleType.REVPH, Constants.PneumaticConstants.kSolenoidForward, Constants.PneumaticConstants.kSolenoidReverse);
    // this.leftClawSolenoid = new NKDoubleSolenoid(2,PneumaticsModuleType.REVPH, Constants.PneumaticConstants.kLeftSolenoidForward, Constants.PneumaticConstants.kLeftSolenoidReverse);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void openClaw(){
    clawSolenoid.set(Value.kForward);
  }

  public void closeClaw(){
    clawSolenoid.set(Value.kReverse);

  }

  public void off(){
    clawSolenoid.set(Value.kOff);
  }

}
