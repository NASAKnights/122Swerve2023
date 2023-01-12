package frc.robot.drive.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.ModuleTest;

public class VoltageTest extends CommandBase {
    private ModuleTest test;
    private double volts;
    public VoltageTest(ModuleTest test, double volts) {
        this.test = test;
        this.volts = volts;
        addRequirements(test);
    }

    @Override
    public void execute() {
        volts = SmartDashboard.getNumber("volts", 0);
        test.setVolts(volts);
    }

    @Override
    public void end(boolean interrupted) {
        test.stop();
    }
}
