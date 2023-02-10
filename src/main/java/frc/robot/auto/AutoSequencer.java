// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.AutoDriveForMeters;
import frc.robot.auto.commands.AutoMoveForSeconds;
import frc.robot.drive.SwerveDrive;


public class AutoSequencer extends SequentialCommandGroup {

  private ChassisSpeeds speeds = new ChassisSpeeds(0.2, 0, 0);

  public AutoSequencer(SwerveDrive swerve) {
    addCommands(
      new AutoMoveForSeconds(swerve, speeds, 3),
      new AutoDriveForMeters(swerve, 1)
      );
  }
}
