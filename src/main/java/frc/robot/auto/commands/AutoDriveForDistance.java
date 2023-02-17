// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;

public class AutoDriveForDistance extends CommandBase {

  private SwerveDrive swerve;
  private ChassisSpeeds speeds;
  private double metersX;
  private double metersY;
  private Rotation2d rotation;
  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidRot;
  private Pose2d desiredPose;
  
  public AutoDriveForDistance(SwerveDrive swerve, double metersX, double metersY, Rotation2d rotation) {
    this.swerve = swerve;
    this.metersX = metersX;
    this.metersY = metersY;
    this.rotation = rotation;
    
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidX = new PIDController(0.5, 0, 0);
    pidY = new PIDController(0.5, 0, 0);
    pidRot = new PIDController(0.1, 0, 0);
    pidX.setTolerance(0.1); 
    pidY.setTolerance(0.1);
    pidRot.setTolerance(0.1);

    swerve.resetHeading();
    swerve.resetDriveEncoders();
    Rotation2d currentRotationTest = swerve.getHeading();
    swerve.resetPose(new Pose2d(0, 0, currentRotationTest));
    //NEED TO FIX RESET

    System.out.println("Initialized \n Initialized \n Initialized \n Initialized \nInitialized \n Initialized \n Initialized \n");

    Translation2d translation = new Translation2d(metersX, metersY);
    Transform2d toMove = new Transform2d(translation, rotation);
    desiredPose = swerve.getPose().transformBy(toMove);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speeds = new ChassisSpeeds(
      pidX.calculate(swerve.getPose().getX(), desiredPose.getX()),
      pidY.calculate(swerve.getPose().getY(), desiredPose.getY()),
      pidRot.calculate(swerve.getPose().getRotation().getDegrees(), desiredPose.getRotation().getDegrees())
    );
    
    // System.out.println(speeds);
    SmartDashboard.putNumber("currentRot", swerve.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("desiredRot", desiredPose.getRotation().getDegrees());
    SmartDashboard.putNumber("rotSpeed", speeds.omegaRadiansPerSecond);
    swerve.drive(speeds, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(), false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(pidX.atSetpoint() == true && pidY.atSetpoint() == true && pidRot.atSetpoint() == true){
      return true;
    }else{
      return false;
    }
  }
}
