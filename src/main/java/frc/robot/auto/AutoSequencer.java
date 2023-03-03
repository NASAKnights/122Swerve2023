// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.SequentialCommands.AutoOutOfCommunity;
import frc.robot.auto.SequentialCommands.AutoRamp;
import frc.robot.auto.commands.AutoDriveForDistance;
import frc.robot.auto.commands.AutoRotateForDegrees;
import frc.robot.auto.commands.TurnForDegrees;
import frc.robot.drive.SwerveDrive;


public class AutoSequencer extends SequentialCommandGroup {

  // private ChassisSpeeds speeds = new ChassisSpeeds(0.2, 0, 0);

  public AutoSequencer(SwerveDrive swerve) {
    //Which sequence would you like today?
    //DISCLAIMER: THIS DOES NOT WORK
    //TODO: Figure out why it does not work
    swerve.resetHeading();
    addCommands(
      new AutoDriveForDistance(swerve, 1, 0, Rotation2d.fromDegrees(0)),
      new TurnForDegrees(swerve, 30, 0.5),
      new AutoDriveForDistance(swerve, 0, 1, Rotation2d.fromDegrees(0))
    );
  }

  //These are all of the presets! Feel free to add some more.
  //Also feel free to call other things such as subsystems, the preset shapes are availible in subsystem form for your convienence too.

  private void Triangle(SwerveDrive swerve){
    //TODO: TRIANGLE FUNCTION (For fun)
  }
  private void Square(SwerveDrive swerve){
    addCommands(
      new AutoDriveForDistance(swerve, 1, 0, Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, 0,1, Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, -1, 0, Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, 0, -1, Rotation2d.fromDegrees(0)),
      new WaitCommand(1)
    );
  }
  private void Hexagon(SwerveDrive swerve){
    addCommands(
      new AutoDriveForDistance(swerve, 0.5, 0, Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, (0.5*Math.cos(Math.PI/3)), (.5 * Math.sin(Math.PI/3)), Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, (-0.5*Math.cos(Math.PI/3)), (.5 * Math.sin(Math.PI/3)), Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, -0.5, 0, Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, (-0.5*Math.cos(Math.PI/3)), (-0.5 * Math.sin(Math.PI/3)), Rotation2d.fromDegrees(0)),
      new WaitCommand(1),
      new AutoDriveForDistance(swerve, (0.5*Math.cos(Math.PI/3)), (-0.2 * Math.sin(Math.PI/3)), Rotation2d.fromDegrees(0))
    );
  }
  private void Hourglass(SwerveDrive swerve){
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


  //NOTE: THIS IS NOT FINISHED (Don't use unless you know what your getting yourself into)
  private void OutOfCommunity(SwerveDrive swerve){
    addCommands(
      new AutoOutOfCommunity(swerve)
    );
  }
  //NOTE: THIS IS NOT FINISHED (Don't use unless you know what your getting yourself into)
  private void AutoRamp(SwerveDrive swerve){
    addCommands(
      new AutoRamp(swerve)
    );
  }
}