// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.drive.SwerveDrive;

public class AutoDriveForMeters extends CommandBase {

  private SwerveDrive swerve;
  private ChassisSpeeds speeds;
  private double meters;
  private PIDController pid;
  
  public AutoDriveForMeters(SwerveDrive swerve, double meters) {
    this.swerve = swerve;
    this.meters = meters;
    
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(0.99, 0, 0);
    pid.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speeds = new ChassisSpeeds(
    pid.calculate(swerve.getDistanceMeters(), 1), 
    0, 
    0);
    if(swerve.getDistanceMeters() < meters){
      if(pid.atSetpoint() == false){
        swerve.drive(speeds, false);
      }else{
        end(isFinished());
      }
    }else{
      end(isFinished());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("over");
    swerve.drive(new ChassisSpeeds(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
