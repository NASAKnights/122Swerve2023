// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.armoutreach.ArmOutreach;

public class LiftToAngle extends CommandBase {
  /** Creates a new LiftToAngle. */
  private ArmOutreach arm;
  private Translation2d targetLocation;
  private boolean finished = false;
  public LiftToAngle(ArmOutreach arm, Translation2d targetLocation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.targetLocation = targetLocation;
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // arm.liftArmtoAngle();
    arm.gotoXY(targetLocation);

    Translation2d xy = arm.getXY();
      Translation2d xyError = targetLocation.minus(xy);

      //check xy error, then continue when xy is within maximum allowed error.
      if(xyError.getNorm() < 0.05){
        finished = true;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
