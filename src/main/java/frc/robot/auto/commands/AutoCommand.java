// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.PhotonVision.PhotonVision;


public class AutoCommand extends CommandBase {
  private SwerveDrive swerve;
  private Timer timer;
  private PhotonVision photon;
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

  public void driveForMeters(ChassisSpeeds speeds, double distance){
    if (swerve.getDistanceMeters() < distance){
      swerve.drive(speeds, false);
    }else{
      end(isFinished());
    }
  }

  public void driveToTarget(double maxSpeed, double distance){
    if (photon.findTarget()){
      double xDist = photon.getXDistanceToTarget();
      double yDist = photon.getYDistanceToTarget();
      double totalDist = photon.distanceFormula(xDist, yDist);
      double xSpeed = (xDist/totalDist) * maxSpeed;
      double ySpeed = (yDist/totalDist) * maxSpeed;

      System.out.println("xSpeed " + xSpeed);
      System.out.println("ySpeed " +ySpeed);
      // if (Math.abs(ySpeed) < .025){
      //   ySpeed = 0;
      // }

      speeds = new ChassisSpeeds(xSpeed,ySpeed,0);

      if (totalDist >= distance){
        swerve.drive(speeds,false);
      }else{
        end(isFinished());
      }
    }
    else{
      end(isFinished());
    }
    

  }

  public void aimAtTarget(double rotationSpeed){
    if (photon.findTarget()){
      double yDist = photon.getYDistanceToTarget();
      double xDist = photon.getXDistanceToTarget();
      double targetAngle = photon.getTargetAngle(xDist, yDist);
      //offset for getHeading needed. Do that next
      double error = swerve.getHeading().getDegrees() - targetAngle;



      speeds = new ChassisSpeeds(0,0,rotationSpeed);

      if (Math.abs(error) > 5){
        swerve.drive(speeds, false);
      }else{
        end(isFinished());
      }

    }
    else{
    
      end(isFinished());
    }
  } 


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    photon = new PhotonVision();
    timer = new Timer();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //ChassisSpeeds speeds = new ChassisSpeeds(0.3, 0, 0);
    aimAtTarget(1);
    //driveToTarget(0.2, 2);
    //driveForSeconds(speeds, 3);

    // SmartDashboard.putBoolean("toleranceStop", pid.atSetpoint());
    // if(pid.atSetpoint() == false){
    //   driveForMeters(speeds, 1);
    // }else{
    //   end(isFinished());
    }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0,0,0), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
