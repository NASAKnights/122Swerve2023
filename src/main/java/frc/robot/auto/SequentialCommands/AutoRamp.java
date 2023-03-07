// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.SequentialCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.AutoDriveWithVelocity;
import frc.robot.drive.SwerveDrive;


public class AutoRamp extends SequentialCommandGroup {

  // private ChassisSpeeds speeds = new ChassisSpeeds(0.2, 0, 0);

  public AutoRamp(SwerveDrive swerve) {
    new Rotation2d();
    swerve.resetHeading();
    addCommands(
      //NOTE: the p value is for testing purposes, I am going to look for a better way of controlling speed soon.
      //TODO: Fine tune and figure out the behavior of p in AutoDriveWithVelocity
      new AutoDriveWithVelocity(swerve, 1, 0, Rotation2d.fromDegrees(0), 0.2)
      );
  }
}
