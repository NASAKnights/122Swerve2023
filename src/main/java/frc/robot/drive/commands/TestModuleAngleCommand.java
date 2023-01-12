package frc.robot.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.ModuleTest;

public class TestModuleAngleCommand extends CommandBase {

    private ModuleTest test;
    private double angle;
    
    public TestModuleAngleCommand(ModuleTest test, double angle) {
        this.test = test;
        this.angle = angle;
    }

    @Override
    public void execute() {
        test.goToAngle(angle);
        System.out.println("TestModuleAngleCommand");
    }

    @Override
    public void end(boolean interrupted) {
        test.stop();
        System.out.println("End TestModuleAngleCommand");
    }

}
