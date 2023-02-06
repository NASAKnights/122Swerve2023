// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;


public class AutoCommand extends CommandBase {
  private SwerveDrive swerve;
  private Timer timer;

  private ChassisSpeeds speeds;

  /** Creates a new AutoTest. */
  public AutoCommand(SwerveDrive swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void driveForSeconds(ChassisSpeeds speeds, double seconds){
    if (timer.get() < seconds){
      swerve.drive(speeds, false);
    }
    else {
      end(isFinished());
    }
  }

  public void driveForMeters(double xSpeed, Double distance){
    double seconds = distance / xSpeed;
    if (timer.get() < seconds){
      speeds = new ChassisSpeeds(xSpeed, 0, 0);
      swerve.drive(speeds, false);
    }else{
      end(isFinished());
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speeds = new ChassisSpeeds(0.3, 0, 0);
    driveForSeconds(speeds, 3);

    //driveForMeters(0.3, 1.0);
      
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}