package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.MathUtil;
import frc.robot.drive.ModuleTest;

public class ModuleTestCommand extends CommandBase {

    private Joystick driver;
    private ModuleTest test;

    public ModuleTestCommand(Joystick driver, ModuleTest test) {
        this.driver = driver;
        this.test = test;
        addRequirements(test);
    }

    @Override
    public void execute() {
        double xSpeed = -MathUtil.calculateAxis(driver.getY(), Constants.kDefaultAxisDeadband, Constants.kMaxTranslationalVelocity);
        double ySpeed = -MathUtil.calculateAxis(driver.getX(), Constants.kDefaultAxisDeadband, Constants.kMaxTranslationalVelocity);
        double thetaSpeed = 0; // = MathUtil.calculateAxis(driver.getZ(), Constants.AXIS_DEADBAND, Constants.MAX_ROTATIONAL_VELOCITY);

        SmartDashboard.putNumber("x", xSpeed);
        SmartDashboard.putNumber("y", ySpeed);

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed);

        // test.setOpenLoop(speeds);
        test.setClosedLoop(speeds);
    }

    @Override
    public void end(boolean interrupted) {
        // test.setOpenLoop(new ChassisSpeeds());
        test.setClosedLoop(new ChassisSpeeds());

    }
}
