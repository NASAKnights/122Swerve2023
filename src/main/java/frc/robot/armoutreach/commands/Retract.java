// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.armoutreach.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.armoutreach.ArmOutreach;
import frc.robot.colorSensor.ColorInterpreter;
import frc.robot.intake.Intake;

public class Retract extends CommandBase {
  /** Creates a new Retract. */
  private ArmOutreach arm;
  private ColorInterpreter indexer;
  private Intake intake;

  private boolean finished = false;
  public Retract(ArmOutreach arm, ColorInterpreter indexer, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.arm = arm;
    this.indexer = indexer;
    this.intake = intake;
    addRequirements(arm);
    addRequirements(indexer);
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (indexer.getLast() == "High Cone") {
      intake.setReverse();
    }
    else if (indexer.getLast() == "Low Cone") {
      intake.setReverse();
    }
    // else if (indexer.checkIndex() == "Cube") {
    //   intake.handOffCube();
    // }
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.retractToZero();
    if (arm.isRetracted()){
      finished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return arm.isRetracted();
    return finished;
  }
}

