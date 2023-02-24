// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.auto.commands.AutoDriveForDistance;
import frc.robot.auto.commands.AutoDriveForSeconds;
import frc.robot.drive.SwerveDrive;


public class AutoHourGlass extends SequentialCommandGroup {

  private ChassisSpeeds speeds = new ChassisSpeeds(0.2, 0, 0);

  public AutoHourGlass(SwerveDrive swerve) {
    new Rotation2d();
    addCommands(
      new AutoDriveForDistance(swerve, 1, 1, Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, -1, 0, Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, 1, -1, Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, -1, 0, Rotation2d.fromDegrees(0)),
      new WaitCommand(1)
      );
  }
}
