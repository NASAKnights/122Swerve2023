// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.SequentialCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.commands.LiftToAngle;
import frc.robot.auto.commands.AutoDriveForDistance;
import frc.robot.drive.SwerveDrive;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.SetIntaketoAngle;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreLow extends SequentialCommandGroup {
  /** Creates a new AutoScoreLow. */
  public AutoScoreLow(SwerveDrive swerve, Intake intake, ArmOutreach arm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new LiftToAngle(arm, new Translation2d(0.1, -0.53)),
      new SetIntaketoAngle(intake, Math.PI * 0.5),
      // new WaitCommand(0.5),
      // new StowIntake(intake),
      new AutoDriveForDistance(swerve, 6.0, 0, Rotation2d.fromDegrees(0))
    );
  }
}
