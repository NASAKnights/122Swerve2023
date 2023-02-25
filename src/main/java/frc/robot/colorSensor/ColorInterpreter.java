// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.colorSensor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorInterpreter extends SubsystemBase {

  private AnalogInput input = new AnalogInput(0);
  /** Creates a new Indexer. */
  public ColorInterpreter() {
  }

  public String checkIndex(){
    double x = ((input.getAverageValue() / 4095.0) * 255);
    if(x < 42){
      SmartDashboard.putString("Whats in my robot", "None");
      return "No Object";
    }
    else if(x < 125){
      SmartDashboard.putString("Whats in my robot", "Cube");
      return "Cube";
    }
    else if (x < 212){
      SmartDashboard.putString("Whats in my robot", "Low Cone");
      return "Low Cone";
    }
    else if (x < 255){
      SmartDashboard.putString("Whats in my robot", "High Cone");
      return "High Cone";
    }
    return "";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
