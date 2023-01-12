package frc.robot.drive.commands;

import frc.robot.drive.SwerveModule;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class TurnSpinnyTest extends CommandBase {
    SwerveModule module;

    public TurnSpinnyTest(SwerveModule module) {
        this.module = module;
    }

    @Override
    public void execute() {
        this.module.testTurnSpinny(0.20);
    }

    @Override
    public void end(boolean interrupted) {
        this.module.testStopSpinny();
    }
}

