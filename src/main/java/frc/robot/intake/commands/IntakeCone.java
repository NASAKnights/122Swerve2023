// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.intake.Intake;

public class IntakeCone extends CommandBase {
  /** Creates a new IntakeCone. */

  private Intake intake;
  private ArmOutreach arm;

  public IntakeCone(Intake intake, ArmOutreach arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.intake = intake;
    addRequirements(intake);
  }
  private boolean armClear;
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armClear = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //after the button has been pressed do the following
    if(!armClear)
    {
      arm.setArmToAngle(4.85);
    }
    if(arm.getArmAngle() > 4.8 && !armClear){
      armClear = true;
    }
    if(armClear)
    {
      intake.setIntakePivot(Math.PI);
      intake.intakeCone();
      arm.setArmToAngle(1.5 * Math.PI);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
    arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
