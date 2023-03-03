// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.claw.Claw;
import frc.robot.colorSensor.ColorInterpreter;
import frc.robot.intake.Intake;

public class HandOff extends CommandBase {
  private ArmOutreach arm;
  private Claw claw;
  private Intake intake;

  private ColorInterpreter indexer;

  private boolean finished = false;

  public HandOff(ArmOutreach arm, Claw claw, Intake intake, ColorInterpreter indexer) {
    this.arm = arm;
    this.claw = claw;
    this.intake = intake;
    this.indexer = indexer;

    addRequirements(arm, claw, intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  // NOTE/DISCLAIMER: The translations need to be tweaked
  @Override
  public void execute() {
    if(indexer.checkIndex() == "Cube"){
      Translation2d targetLocation = new Translation2d(-0.15, -0.622);

      arm.gotoXY(targetLocation);

      Translation2d xy = arm.getXY();
      Translation2d xyError = targetLocation.minus(xy);

      //check xy error, then continue when xy is within maximum allowed error.
      if(xyError.getNorm() < 0.05){
        claw.closeClaw();
        finished = true;
      }
    }
    else if(indexer.checkIndex() == "Low Cone"){
      Translation2d targetLocation = new Translation2d(-0.15, -0.622);

      arm.gotoXY(targetLocation);
      // claw.openClaw();

      Translation2d xy = arm.getXY();
      Translation2d xyError = targetLocation.minus(xy);

      //check xy error, then continue when xy is within maximum allowed error.
      if(xyError.getNorm() < 0.01){
        claw.closeClaw();
        finished = true;
      }
    }
    else if(indexer.checkIndex() == "High Cone"){
      Translation2d targetLocation = new Translation2d(0.0728, -0.6801);

      arm.gotoXY(targetLocation);
      // claw.openClaw();

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
      
      if (indexer.checkIndex() == "Cube"){
        intake.handOffCube();
      }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}