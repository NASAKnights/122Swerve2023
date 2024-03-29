// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.armoutreach.ArmOutreach;

public class GoToHigh extends CommandBase {
  /** Creates a new GoToHigh. */
  private ArmOutreach arm;
  private boolean isFinished;
  private boolean isCube;

  public GoToHigh(ArmOutreach arm, boolean isCube) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.isCube = isCube;

    addRequirements(arm);
  }

  public GoToHigh(ArmOutreach arm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.isCube = false;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // 877, 205
    // Translation2d targetLocation = new Translation2d(0.9,0.2);
    double max = Constants.ArmConstants.kExtensionLength + Constants.ArmConstants.kExtensionRetractedLength;
    Translation2d targetLocation;
    if(isCube){
      targetLocation = new Translation2d(max, Rotation2d.fromDegrees(365));
    }
    else{
      targetLocation = new Translation2d(max, Rotation2d.fromDegrees(380));
    }

    arm.gotoXY(targetLocation);

    Translation2d xy = arm.getXY();
    Translation2d xyError = targetLocation.minus(xy);

    if(xyError.getNorm() < 0.02){
      isFinished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
    arm.stopArm();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
