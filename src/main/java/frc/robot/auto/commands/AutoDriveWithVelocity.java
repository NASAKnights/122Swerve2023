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

public class AutoDriveWithVelocity extends CommandBase {

  private SwerveDrive swerve;
  private ChassisSpeeds speeds;

  private double metersX;
  private double metersY;
  private Rotation2d rotation;

  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidRot;
  private Pose2d desiredPose;

  private double velocityLowerLimit = 0.2;
  private double rotationalLowerLimit = 0.2;

  private double maxVelocity = 1;
  
  public AutoDriveWithVelocity(SwerveDrive swerve, double metersX, double metersY, Rotation2d rotation, double maxVelocity) {
    this.swerve = swerve;
    this.metersX = metersX;
    this.metersY = metersY;
    this.rotation = rotation;
    this.maxVelocity = maxVelocity;
    
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidX = new PIDController(maxVelocity, 0, 0);
    pidY = new PIDController(maxVelocity, 0, 0);
    pidRot = new PIDController(0.15, 0, 0);
    pidX.setTolerance(0.05); 
    pidY.setTolerance(0.05);
    pidRot.setTolerance(0.1);

    //If the rotational tolerance is set too low it tends to make the robot squeak 
    //(Side Note: Positional tolerance also makes the robot squeak if its too low but its more 'tolerable') 

    Rotation2d currentRotationTest = swerve.getHeading();
    // swerve.resetHeading();
    //swerve.resetDriveEncoders();
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
    RobotDeadzone();
    
    // System.out.println(speeds);
    SmartDashboard.putNumber("currentRot", swerve.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("desiredRot", desiredPose.getRotation().getDegrees());
    SmartDashboard.putNumber("rotSpeed", speeds.omegaRadiansPerSecond);
    System.out.println(speeds);
    swerve.drive(speeds, false);
  }

  private void RobotDeadzone() {
    //This is most likely not the best way of doing this.
    if(speeds.vxMetersPerSecond <= velocityLowerLimit){
      speeds.vxMetersPerSecond = 0;
    }
    if(speeds.vyMetersPerSecond <= velocityLowerLimit){
      speeds.vyMetersPerSecond = 0;
    }
    if(speeds.omegaRadiansPerSecond <= rotationalLowerLimit){
      speeds.omegaRadiansPerSecond = 0;
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
    if(pidX.atSetpoint() == true && pidY.atSetpoint() == true && pidRot.atSetpoint() == true){
      return true;
    }else{
      return false;
    }
  }
}
