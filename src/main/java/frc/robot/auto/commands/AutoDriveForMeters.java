// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;

<<<<<<< HEAD:src/main/java/frc/robot/auto/commands/AutoDriveForMeters.java
public class AutoDriveForMeters extends CommandBase {

  private SwerveDrive swerve;
  private ChassisSpeeds speeds;
  private double meters;
  private PIDController pid;
  
  public AutoDriveForMeters(SwerveDrive swerve, double meters) {
    this.swerve = swerve;
    this.meters = meters;
    
=======
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
>>>>>>> bd875eb6514cb9c8d11bf2c05dc90beb929a4545:src/main/java/frc/robot/auto/commands/TurnForDegrees.java
    addRequirements(swerve);
    this.degrees = degrees;
    this.rotationSpeed = rotationSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
<<<<<<< HEAD:src/main/java/frc/robot/auto/commands/AutoDriveForMeters.java
    pid = new PIDController(0.99, 0, 0);
    pid.setTolerance(0.01);
=======
    System.out.println("initialize");
    initialPosition = swerve.getHeading().getDegrees();
    finalPosition = initialPosition + degrees;
>>>>>>> bd875eb6514cb9c8d11bf2c05dc90beb929a4545:src/main/java/frc/robot/auto/commands/TurnForDegrees.java
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
<<<<<<< HEAD:src/main/java/frc/robot/auto/commands/AutoDriveForMeters.java
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
=======
    
    if (degrees < 0 ){
      speeds = new ChassisSpeeds(0,0,-rotationSpeed);
    }else{
      speeds = new ChassisSpeeds(0,0,rotationSpeed);
    }
  
    if (swerve.getHeading().getDegrees() >= finalPosition + 5 || swerve.getHeading().getDegrees() <= finalPosition -5){
      swerve.drive(speeds, false);
    }
    else{
>>>>>>> bd875eb6514cb9c8d11bf2c05dc90beb929a4545:src/main/java/frc/robot/auto/commands/TurnForDegrees.java
      end(isFinished());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
<<<<<<< HEAD:src/main/java/frc/robot/auto/commands/AutoDriveForMeters.java
    System.out.println("over");
    swerve.drive(new ChassisSpeeds(), false);
=======
    swerve.drive(new ChassisSpeeds(0,0,0), false);
>>>>>>> bd875eb6514cb9c8d11bf2c05dc90beb929a4545:src/main/java/frc/robot/auto/commands/TurnForDegrees.java
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
