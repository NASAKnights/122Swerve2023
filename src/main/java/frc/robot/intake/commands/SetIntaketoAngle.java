// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;

public class SetIntaketoAngle extends CommandBase {
  /** Creates a new SetIntaketoAngle. */
  private Intake intake;
  private double angle; //in radians
  public SetIntaketoAngle(Intake intake, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.angle = angle;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakePivot(angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return false;
    if (Math.abs(intake.checkAngle() - angle) < 0.5){
      return true;
    }
    return false;
    
  }
}
