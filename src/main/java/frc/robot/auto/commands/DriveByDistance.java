// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.drive.SwerveDrive;

public class DriveByDistance extends CommandBase {
  /** Creates a new DriveByDistance. */
  private SwerveDrive swerve;
  private double xMeters;
  private double yMeters;
  private double rotRadians;

  private PIDController xControl;
  private PIDController yControl;
  private PIDController angleControl;

  public DriveByDistance(SwerveDrive swerve, double x, double y, double rot) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.swerve = swerve;
    this.xMeters = x;
    this.yMeters = y;
    this.rotRadians = rot;

    xControl = new PIDController(0.5, 0, 0);
    yControl = new PIDController(0.5, 0, 0);
    angleControl = new PIDController(0.5, 0, 0);

    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetDriveEncoders();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ChassisSpeeds speeds = new ChassisSpeeds();
    swerve.drive(speeds, Constants.kIsOpenLoop);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(), Constants.kIsOpenLoop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
