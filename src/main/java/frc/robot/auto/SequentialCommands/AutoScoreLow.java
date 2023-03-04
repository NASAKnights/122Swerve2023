// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.SequentialCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.AutoDriveForDistance;
import frc.robot.drive.SwerveDrive;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.SetIntakeForward;
import frc.robot.intake.commands.SetIntaketoAngle;
import frc.robot.intake.commands.StowIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoScoreLow extends SequentialCommandGroup {
  /** Creates a new AutoScoreLow. */
  public AutoScoreLow(SwerveDrive swerve, Intake intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetIntaketoAngle(intake, Math.PI),
      new WaitCommand(0.5),
      new ParallelCommandGroup(
        new AutoDriveForDistance(swerve, 4, 0, null),  
        new StowIntake(intake)
      )
    );
  }
}
