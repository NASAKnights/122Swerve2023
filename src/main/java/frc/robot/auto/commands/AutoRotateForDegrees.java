// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.drive.SwerveDrive;

public class AutoRotateForDegrees extends CommandBase {
  
  private SwerveDrive swerve;
  private double degrees;
  private PIDController pid;

  public AutoRotateForDegrees(SwerveDrive swerve, double degrees) {
    this.swerve = swerve;
    this.degrees = degrees;

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pid = new PIDController(0, 0, 0);
    pid.setTolerance(0.01);
    double desiredDouble = swerve.getHeading().getDegrees() + degrees;
    Rotation2d desiredDegrees = new Rotation2d().fromDegrees(desiredDouble);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(null, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
