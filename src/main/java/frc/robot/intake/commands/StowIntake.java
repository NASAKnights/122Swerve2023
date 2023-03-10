// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.intake.Intake;

public class StowIntake extends CommandBase {

  private Intake intake;
  private ArmOutreach arm;
  private boolean finished = false;
  private boolean intakeVert = false;
  /** Creates a new StowIntake. */
  public StowIntake(Intake intake, ArmOutreach arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.arm = arm;
    addRequirements(intake);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(intake.getAngle() < 0.05)
    {
      finished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setIntakePivot(0);
    if(intake.getAngle() < Math.PI/2 && !intakeVert)
    {
      intakeVert = true;
    }
    else
    {
      arm.setArmToAngle((20.0 * Math.PI) / 12.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntakeLift();
    arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
    
  }
}
