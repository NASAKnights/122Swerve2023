// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.SequentialCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.commands.StowInside;
import frc.robot.auto.commands.AutoDriveForDistance;
import frc.robot.claw.Claw;
import frc.robot.drive.SwerveDrive;
import frc.robot.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreMidLong extends SequentialCommandGroup {
  /** Creates a new AutoScoreMidBalance. */
  public AutoScoreMidLong(SwerveDrive swerve, Intake intake, ArmOutreach arm, Claw claw) {
    swerve.resetHeading();
    addCommands(
                new AutoScoreMid(swerve, intake, arm, claw),
                new ParallelCommandGroup(new StowInside(arm),
                                         new AutoDriveForDistance(swerve, -5, 0, new Rotation2d()))
                // new AutoDriveWithVelocity(swerve, 0, 0, Rotation2d.fromDegrees(180), 0.7),
                // new InstantCommand(swerve::resetHeading)
                );
  }
}

