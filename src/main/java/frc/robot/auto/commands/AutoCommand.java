// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;
import edu.wpi.first.math.controller.PIDController;


public class AutoCommand extends CommandBase {
  private SwerveDrive swerve;
  private Timer timer;

  /** Creates a new AutoTest. */
  public AutoCommand(SwerveDrive swerve) {
    this.swerve = swerve;

    
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void driveForSeconds(ChassisSpeeds speeds, double seconds){
    if (timer.get() < seconds){
      swerve.drive(speeds, false);
    }else {
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
    pid = new PIDController(0.99, 0, 0);
    pid.setTolerance(0.01);

    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("PID" + pid.calculate(swerve.getDistanceMeters(), 1));
    ChassisSpeeds speeds = new ChassisSpeeds(pid.calculate(swerve.getDistanceMeters(), 1), 0, 0);

    if(pid.atSetpoint() == false){
      driveForMeters(speeds, 1);
    }else{
      end(isFinished());
    }
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
