// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.SequentialCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.commands.StowInside;
import frc.robot.auto.commands.AutoBalance;
import frc.robot.auto.commands.CustomAutoDriveForDistance;
import frc.robot.claw.Claw;
import frc.robot.claw.commands.OpenClaw;
import frc.robot.drive.SwerveDrive;
import frc.robot.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreHighBalance extends SequentialCommandGroup {
  /** Creates a new AutoScoreHighBalance. */
  public AutoScoreHighBalance(SwerveDrive swerve, Intake intake, ArmOutreach arm, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    swerve.resetHeading();
    addCommands(
                new OpenClaw(claw),
                new ParallelCommandGroup(new StowInside(arm),
                                         new CustomAutoDriveForDistance(swerve, -3.04, 0, new Rotation2d(), 0.7)),
                                         
                // new AutoRotateForDegrees(swerve, 1),
                new AutoBalance(swerve),
                
                //Invert the heading for drive team
                new InstantCommand(swerve::invertHeading)
                // new InstantCommand(swerve::resetHeading)
                );
  }
}