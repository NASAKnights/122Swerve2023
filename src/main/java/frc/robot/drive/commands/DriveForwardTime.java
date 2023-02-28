// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.drive.SwerveDrive;

public class DriveForwardTime extends WaitCommand {
  /** Creates a new DriveForwardTime. */

  private SwerveDrive swerve;

  public DriveForwardTime(SwerveDrive swerve, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(time);

    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    super.initialize();
    swerve.resetHeading();
    swerve.drive(new ChassisSpeeds(0.7, 0,0), Constants.kIsOpenLoop);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    swerve.drive(new ChassisSpeeds(), Constants.kIsOpenLoop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return super.isFinished();
  }
}
