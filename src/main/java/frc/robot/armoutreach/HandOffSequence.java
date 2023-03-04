// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.armoutreach.commands.HandOff;
import frc.robot.armoutreach.commands.Retract;
import frc.robot.claw.Claw;
import frc.robot.claw.commands.OpenClaw;
import frc.robot.colorSensor.ColorInterpreter;
import frc.robot.intake.Intake;
import frc.robot.intake.commands.StopIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HandOffSequence extends SequentialCommandGroup {
  
  /** Creates a new handoffSequence. */
  public HandOffSequence(ArmOutreach arm, Intake intake, Claw claw, ColorInterpreter indexer) {
  
    addCommands(
      new OpenClaw(claw),
      new WaitCommand(0.25),
      new HandOff(arm, claw, intake, indexer),
      new WaitCommand(0.25),
      new Retract(arm, indexer, intake),
      new StopIntake(intake)
    );
  }
}
