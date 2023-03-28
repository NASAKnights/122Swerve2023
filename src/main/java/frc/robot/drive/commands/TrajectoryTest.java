// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.commands;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;

public class TrajectoryTest extends CommandBase {
  /** Creates a new TrajectoryTest. */
  Trajectory traj;
  SwerveDrive swerve;
  public TrajectoryTest(SwerveDrive swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    traj = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
      
      List.of(
        new Translation2d(1, 0),
        new Translation2d(0, 1)),
      
      new Pose2d(1,1, Rotation2d.fromDegrees(0)),
      new TrajectoryConfig(2, 2));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    new FollowPath(swerve);
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
