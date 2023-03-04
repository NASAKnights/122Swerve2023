// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.Intake;

public class StowIntake extends CommandBase {

  private Intake intake;
  private boolean finished = false;
  /** Creates a new StowIntake. */
  public StowIntake(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // intake.stowIntake();
    intake.lowerIntake();
    if (Math.abs(intake.checkAngle()) < 0.5){
      finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeLift();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
    
  }
}
