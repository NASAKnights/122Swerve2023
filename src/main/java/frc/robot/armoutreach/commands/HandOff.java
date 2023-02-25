// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach.commands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.claw.Claw;
import frc.robot.indexer.Indexer;
import frc.robot.intake.Intake;

public class HandOff extends CommandBase {
  private ArmOutreach arm;
  private Claw claw;
  private Indexer indexer;
  private boolean finished = false;
  private Intake intake;
  /** Creates a new handoffCube. */
  public HandOff() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(indexer.checkIndex() == "Cube"){
      Translation2d targetLocation = new Translation2d(-1, 0);

      arm.gotoXY(targetLocation);

      Translation2d xy = arm.getXY();
      Translation2d xyError = targetLocation.minus(xy);

      //check xy error, then continue when xy is within maximum allowed error.
      if(xyError.getNorm() < 0.01){
        claw.closeClaw();
        finished = true;
      }
    }
    else if(indexer.checkIndex() == "Low Cone"){
      Translation2d targetLocation = new Translation2d(-1, 0);

      arm.gotoXY(targetLocation);
      claw.openClaw();

      Translation2d xy = arm.getXY();
      Translation2d xyError = targetLocation.minus(xy);

      //check xy error, then continue when xy is within maximum allowed error.
      if(xyError.getNorm() < 0.01){
        claw.closeClaw();
        finished = true;
      }
    }
    else if(indexer.checkIndex() == "High Cone"){
      Translation2d targetLocation = new Translation2d(-1, 0);

      arm.gotoXY(targetLocation);
      claw.openClaw();

      Translation2d xy = arm.getXY();
      Translation2d xyError = targetLocation.minus(xy);

      

      //check xy error, then continue when xy is within maximum allowed error.
      if(xyError.getNorm() < 0.01){
        claw.closeClaw();
        finished = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
      intake.setReverse();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}