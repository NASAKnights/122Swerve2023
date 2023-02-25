// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.armoutreach.ArmOutreach;

public class GoToHigh extends CommandBase {
  /** Creates a new GoToHigh. */
  private ArmOutreach arm;
  public GoToHigh(ArmOutreach arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // int object = arm.checkObject();
    // if (object == 0){ // Cube
    //   arm.gotoXY(Constants.kCubeHigh);
    // }
    // else if (object == 1){ // Cone inside
    //   arm.gotoXY(Constants.kConeHigh);
    // }
    // else { // Cone Outside
    //   arm.gotoXY(Constants.kConeHigh);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
