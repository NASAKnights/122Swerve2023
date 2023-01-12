package frc.robot.drive.commands;

import frc.robot.drive.SwerveModule;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class DriveSpinnyTest extends CommandBase {
    SwerveModule module;

    public DriveSpinnyTest(SwerveModule module) {
        this.module = module;
    }

    @Override
    public void execute() {
        this.module.testDriveSpinny(0.20);
    }

    @Override
    public void end(boolean interrupted) {
        this.module.testStopSpinny();
    }
}

