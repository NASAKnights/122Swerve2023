// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.armoutreach.commands.Retract;
import frc.robot.claw.Claw;
import frc.robot.claw.commands.OpenClaw;
import frc.robot.armoutreach.commands.HandOff;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.SetIntakeReverse;
import frc.robot.intake.commands.StopIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class handoffSequence extends SequentialCommandGroup {
  private ArmOutreach arm;
  private Intake intake;
  private Claw claw;
  /** Creates a new handoffSequence. */
  public handoffSequence() {
    addCommands(
      new OpenClaw(claw),
      new HandOff(),
      new Retract(arm),
      new StopIntake()
    );
  }
}