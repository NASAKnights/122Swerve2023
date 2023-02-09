// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.PhotonVision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera camera;

  public PhotonVision() {
    
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  }

  private int getCameraData(PhotonCamera camera){
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    if (hasTargets == true){
      //List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();

      int targetID = target.getFiducialId();

      return targetID;
    }

    return 0;


  }


  public void updateSmartDash(){

    SmartDashboard.putNumber("Target ID", getCameraData(camera));
    //SmartDashboard.putBoolean("hasTargets",);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
