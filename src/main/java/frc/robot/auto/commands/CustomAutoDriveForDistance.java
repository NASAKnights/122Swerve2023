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

public class CustomAutoDriveForDistance extends CommandBase {

  private SwerveDrive swerve;
  private ChassisSpeeds speeds;

  private double metersX;
  private double metersY;
  private Rotation2d rotation;

  private PIDController pidX;
  private PIDController pidY;
  private PIDController pidRot;
  private Pose2d desiredPose;
  private boolean finished = false;

  private double velocityLowerLimit = 0.05;
  private double rotationalLowerLimit = 0.1;

  private double p = 1;
  
  /** Please choose AutoDriveForDistance instead of this unless this is needed, this command was made for one purpose,
   * to run on the AutoScore command(s) and as such may cause weird behavior when used in other conditions */
  public CustomAutoDriveForDistance(SwerveDrive swerve, double metersX, double metersY, Rotation2d rotation, double p) {
    this.swerve = swerve;
    this.metersX = metersX;
    this.metersY = metersY;
    this.rotation = rotation;
    this.p = p;
    
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Maybe add pid reset function here(???)
    pidX = new PIDController(p, 0, 0);
    pidY = new PIDController(p, 0, 0);
    pidRot = new PIDController(0.05, 0, 0.05);
    pidX.setTolerance(0.05); 
    pidY.setTolerance(0.05);
    pidRot.setTolerance(5); // degrees

    speeds = new ChassisSpeeds();

    Rotation2d currentRotationTest = swerve.getHeading();
    // swerve.resetHeading();
    // swerve.resetDriveEncoders();
    swerve.resetPose(new Pose2d(0, 0, currentRotationTest));

    Translation2d translation = new Translation2d(metersX, metersY);
    Transform2d toMove = new Transform2d(translation, rotation);
    desiredPose = swerve.getPose().transformBy(toMove);
    // speeds.vxMetersPerSecond = velocityLowerLimit;

    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    speeds = new ChassisSpeeds(
      pidX.calculate(swerve.getPose().getX(), desiredPose.getX()),
      pidY.calculate(swerve.getPose().getY(), desiredPose.getY()),
      0
      //pidRot.calculate(swerve.getPose().getRotation().getDegrees(), desiredPose.getRotation().getDegrees())
    );

    RobotDeadzone();

    SmartDashboard.putNumber("currentRot", swerve.getPose().getRotation().getDegrees());
    SmartDashboard.putNumber("desiredRot", desiredPose.getRotation().getDegrees());
    SmartDashboard.putNumber("rotSpeed", speeds.omegaRadiansPerSecond);

    // System.out.println(speeds);

    swerve.drive(speeds, false);
  }

  private void RobotDeadzone() {
    //This is most likely not the best way of doing this.
    if(Math.abs(speeds.vxMetersPerSecond) < velocityLowerLimit){
      speeds.vxMetersPerSecond = 0;
    }
    if(Math.abs(speeds.vyMetersPerSecond) < velocityLowerLimit){
      speeds.vyMetersPerSecond = 0;
    }
    if(Math.abs(speeds.omegaRadiansPerSecond) < rotationalLowerLimit){
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
    if(speeds.vxMetersPerSecond == 0 && speeds.vyMetersPerSecond == 0 && speeds.omegaRadiansPerSecond == 0){
      // System.out.println("Yes I am done moving");
      // return true;
      finished = true;
    }
    // return false;
    return finished;
  }
}
