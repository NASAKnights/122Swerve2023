package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;

public class VoltageTest extends CommandBase {
    // private ModuleTest test;
    private SwerveDrive swerve;
    private double mps;
    public VoltageTest(SwerveDrive swerve, double mps) {
        this.swerve = swerve;
        this.mps = mps;
        addRequirements(swerve);
        SmartDashboard.putNumber("Velocity open loop", 0);
    }

    @Override
    public void execute() {
        mps = SmartDashboard.getNumber("Velocity open loop", 0);
        // test.setVolts(volts);
        swerve.drive(new ChassisSpeeds(mps, 0, 0), true);
    }

    @Override
    public void end(boolean interrupted) {
        // test.stop();
        swerve.drive(new ChassisSpeeds(), true);
    }
}
