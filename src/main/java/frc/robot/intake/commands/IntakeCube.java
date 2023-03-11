// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.intake.Intake;

public class IntakeCube extends CommandBase {
  /** Creates a new IntakeCube. */

  private Intake intake;
  private ArmOutreach arm;
  private boolean armHasReachedOut = false;
  private int stage = 1;

  public IntakeCube(Intake intake, ArmOutreach arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Stage1, Move arm out of the way
    //Stage2, Move intake out AND move the arm in
    if(stage == 1){
      arm.setArmToAngle(4.85);
      if(arm.getArmAngle() > 4.8){
        stage++;
      }
    }
    else if(stage == 2){
      intake.setIntakePivot(2.65);
      arm.setArmToAngle(4.712);
      intake.intakeCube();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
