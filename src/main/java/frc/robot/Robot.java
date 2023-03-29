
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Robot extends TimedRobot {

    private RobotContainer container;
    private CommandBase autoSequencer;

    private CommandBase m_autoCommand;

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
        m_autoCommand = container.autonomousInit();
        if (m_autoCommand != null){
            m_autoCommand.schedule();
        }
        else{
            System.out.println("No Auto Found");
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        if (m_autoCommand != null){
            m_autoCommand.cancel();
        }
        container.teleopInit();
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
