package frc.robot.drive.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;

public class VelocityTest extends CommandBase {
    private SwerveDrive swerve;
    private double mps;
    public VelocityTest(SwerveDrive swerve, double mps) {
        this.swerve = swerve;
        this.mps = mps;
        addRequirements(swerve);
        SmartDashboard.putNumber("Target velocity", 0.0);
    }

    @Override
    public void execute() {
        mps = SmartDashboard.getNumber("Target velocity", 0.0);
        // test.setMPS(mps);
        
        swerve.drive(new ChassisSpeeds(mps, 0, 0), false);
    }

    @Override
    public void end(boolean interrupted) {
        // test.stop();
        swerve.drive(new ChassisSpeeds(), false);
    }
}
