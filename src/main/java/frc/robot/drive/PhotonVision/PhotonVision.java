// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.PhotonVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera camera;
  

  public PhotonVision() {
    
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  }

  private boolean findTarget(){
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    return hasTargets;

  }

  private int getTagID(){
    var result = camera.getLatestResult();
  
    if (findTarget()){
      //List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();

      int targetID = target.getFiducialId();

      return targetID;
    }
    return 0;
  }

  private double getDistanceToTarget(){
    var result = camera.getLatestResult();

    //Camera and Target Positions
    double cameraHeightMeters = Units.inchesToMeters(33 + (7/32));
    double targetHeightMeters = Units.inchesToMeters(39 + (3/16));
    double cameraPitchRadians = .108;



    if (findTarget()){
      double range = PhotonUtils.calculateDistanceToTargetMeters(
                                                  cameraHeightMeters,
                                                  targetHeightMeters,
                                                  cameraPitchRadians,
                                                  Units.degreesToRadians(result.getBestTarget().getPitch()));

      //System.out.println(Units.degreesToRadians(result.getBestTarget().getPitch()));

      return range;
    }

    return 0;

  }


  private double getPitch(){
    var result = camera.getLatestResult();

    if (findTarget()){
      PhotonTrackedTarget target = result.getBestTarget();

      double pitch = target.getPitch();

      return pitch;

    }
    return 0;
  }



  public void updateSmartDash(){

    SmartDashboard.putBoolean("hasTargets",findTarget());
    SmartDashboard.putNumber("Target ID", getTagID());
    SmartDashboard.putNumber("Pitch (Deg)", getPitch());
    SmartDashboard.putNumber("Target Distance", getDistanceToTarget());
   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
