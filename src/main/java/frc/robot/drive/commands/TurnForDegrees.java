// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import java.io.ObjectInputStream.GetField;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;

public class TurnForDegrees extends CommandBase {
  /** Creates a new TurnForDegrees. */
  private SwerveDrive swerve;
  private ChassisSpeeds speeds;
  private double degrees;
  private double finalPosition;
  private double rotationSpeed;
  private double initialPosition;


  public TurnForDegrees(SwerveDrive swerve, double degrees, double rotationSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    addRequirements(swerve);
    this.degrees = degrees;
    this.rotationSpeed = rotationSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("initialize");
    initialPosition = swerve.getHeading().getDegrees();
    finalPosition = initialPosition + degrees;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (degrees < 0 ){
      speeds = new ChassisSpeeds(0,0,-rotationSpeed);
    }else{
      speeds = new ChassisSpeeds(0,0,rotationSpeed);
    }
  

    if (swerve.getHeading().getDegrees() >=  finalPosition + 5 || swerve.getHeading().getDegrees() <= finalPosition -5){
      
      swerve.drive(speeds, false);
    }
    else{
      end(isFinished());
    }
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
