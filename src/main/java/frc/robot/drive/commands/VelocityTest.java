package frc.robot.drive.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.ModuleTest;

public class VelocityTest extends CommandBase {
    private ModuleTest test;
    private double mps;
    public VelocityTest(ModuleTest test, double mps) {
        this.test = test;
        this.mps = mps;
        addRequirements(test);
        SmartDashboard.putNumber("Target velocity", 0.0);
    }

    @Override
    public void execute() {
        mps = SmartDashboard.getNumber("Target velocity", 0.0);
        test.setMPS(mps);
    }

    @Override
    public void end(boolean interrupted) {
        test.stop();
    }
}
