
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private RobotContainer container;
    private CommandBase autoCommand;

    @Override
    public void robotInit() {
        container = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        container.periodic();
    }

    @Override
    public void autonomousInit() {
        autoCommand = container.autonomousInit();
        if (autoCommand != null){
            autoCommand.schedule();
        }
       // container.autonomousInit();
    }

    @Override
    public void autonomousPeriodic() {
       
    }

    @Override
    public void teleopInit() {
        container.teleopInit();
        if (autoCommand != null){
            autoCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void disabledInit() {
        container.disabledInit();
    }

    @Override
    public void disabledPeriodic() {
        container.periodic();
        container.disabledPeriodic();

    }

    @Override
    public void testInit() {
        container.testInit();
    }

    @Override
    public void testPeriodic() {
        container.testPeriodic();
    }

    @Override
    public void testExit() {
        container.testExit();
    }

    @Override
    public void simulationInit() {
    }

    @Override
    public void simulationPeriodic() {
    }
}
