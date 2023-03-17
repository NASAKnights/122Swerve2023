// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.SequentialCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.commands.GoToHigh;
import frc.robot.armoutreach.commands.StowInside;
import frc.robot.auto.commands.AutoDriveForDistance;
import frc.robot.claw.Claw;
import frc.robot.claw.commands.OpenClaw;
import frc.robot.drive.SwerveDrive;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.StowIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreHighBalance extends SequentialCommandGroup {
  /** Creates a new AutoScoreHighBalance. */
  public AutoScoreHighBalance(SwerveDrive swerve, Intake intake, ArmOutreach arm, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    swerve.resetHeading();
    addCommands(new GoToHigh(arm),
                new AutoDriveForDistance(swerve, 0.7, 0, new Rotation2d()),
                new OpenClaw(claw),
                
                new ParallelCommandGroup(new AutoDriveForDistance(swerve, -2.54, 0, new Rotation2d()),
                                         new StowInside(arm)
                  ));
  }
}


