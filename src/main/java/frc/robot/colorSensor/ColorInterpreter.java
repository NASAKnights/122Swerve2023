// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.colorSensor;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorInterpreter extends SubsystemBase {

  private AnalogInput input = new AnalogInput(0);

  private String item = "none";
  private String lastItem = "none";
  /** Creates a new Indexer. */
  public ColorInterpreter() {
  }

  public String getLast(){
    return lastItem;
  }
  public void setLast(String item){
    lastItem = item;
  }

  public String checkIndex(){

    double scaling = 1.25;
    double x = scaling * ((input.getAverageValue() / 4095.0) * 255);
    if(x < 42){
      SmartDashboard.putString("Whats in my robot", "None");
      return "none";
    }
    else if(x < 125){
      SmartDashboard.putString("Whats in my robot", "Cube");
      return "cube";
    }
    else if (x < 255){
      SmartDashboard.putString("Whats in my robot", "Cone");
      return "cone";
    }
    return "";
  }

  public void setItem(){
    item = checkIndex();

  }
  public String checkItem() {
    return item;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
