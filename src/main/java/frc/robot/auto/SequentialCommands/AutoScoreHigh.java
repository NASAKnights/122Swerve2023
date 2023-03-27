// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.SequentialCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.commands.GoToHigh;
import frc.robot.armoutreach.commands.LowerArm;
import frc.robot.auto.commands.AutoDriveForDistance;
import frc.robot.claw.Claw;
import frc.robot.claw.commands.CloseClaw;
import frc.robot.claw.commands.OpenClaw;
import frc.robot.drive.SwerveDrive;
import frc.robot.intake.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreHigh extends SequentialCommandGroup {
  /** NOTE: This is used for closing the claw, making the arm go high, driving forwards, and then scoring a cone or cube in auto */
  public AutoScoreHigh(SwerveDrive swerve, Intake intake, ArmOutreach arm, Claw claw) {
    swerve.resetHeading();
    addCommands(
      new CloseClaw(claw),
      new GoToHigh(arm),
      new ParallelRaceGroup(new WaitCommand(4),
                            new AutoDriveForDistance(swerve, 0.75, 0, new Rotation2d())),
      new WaitCommand(2),
      new ParallelDeadlineGroup(new WaitCommand(0.5),
                                new LowerArm(arm)),
      new OpenClaw(claw)
                );
  }
}
