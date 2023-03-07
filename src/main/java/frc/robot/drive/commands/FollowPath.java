package frc.robot.drive.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.SwerveDrive;

public class FollowPath extends CommandBase {

    SwerveDrive swerve;
    Trajectory trajectory;
    Timer timer;

    PIDController xController, yController, thetaController;

    final double kTranslationalP = 0, kTranslationalI = 0, kTranslationalD = 0;
    final double kRotationalP = 0, kRotationalI = 0, kRotationalD = 0; 
    
    public FollowPath(SwerveDrive swerve, Trajectory traj) {
        this.swerve = swerve;
        this.trajectory = traj;
        this.timer = new Timer();

        this.xController = new PIDController(kTranslationalP, kTranslationalI, kTranslationalD);
        this.yController = new PIDController(kTranslationalP, kTranslationalI, kTranslationalD);
        this.thetaController = new PIDController(kRotationalP, kRotationalI, kRotationalD);
        this.thetaController.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        swerve.resetPose(trajectory.getInitialPose()); // TODO doesn't work if you use vision
    }
    
    @Override
    public void execute() {
        double xDot, yDot, thetaDot;
        Pose2d targetPose = trajectory.sample(getCommandTime()).poseMeters;
        Pose2d pos = swerve.getPose();

        xDot = xController.calculate(pos.getX(), targetPose.getX());
        yDot = yController.calculate(pos.getY(), targetPose.getY());
        thetaDot = thetaController.calculate(pos.getRotation().getDegrees(), targetPose.getRotation().getDegrees());

        ChassisSpeeds speeds = new ChassisSpeeds(xDot, yDot, thetaDot);

        swerve.drive(speeds);
    }
    
    @Override
    public void end(boolean interrupted) {
        timer.stop();
    }
    
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(trajectory.getTotalTimeSeconds());
    }

    private double getCommandTime() {
        return timer.get();
    }
}
