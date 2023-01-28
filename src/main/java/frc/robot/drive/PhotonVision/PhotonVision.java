// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drive.PhotonVision;

import java.util.List;
import java.lang.Math;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import pabeles.concurrency.ConcurrencyOps.NewInstance;



public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  PhotonCamera camera;

  public PhotonVision() {
    
    camera = new PhotonCamera("Microsoft_LifeCam_HD-3000");

  }

  public boolean findTarget(){
    // locates targets, returns true when target is found
    var result = camera.getLatestResult();
    boolean hasTargets = result.hasTargets();

    return hasTargets;

  }

  private int getTagID(){
    // returns tag ID
    var result = camera.getLatestResult();
  
    if (findTarget()){
      //List<PhotonTrackedTarget> targets = result.getTargets();
      PhotonTrackedTarget target = result.getBestTarget();

      int targetID = target.getFiducialId();

      return targetID;
    }
    return 0;
  }

  public double getXDistanceToTarget(){
    // gets foward distance in meters
    var result = camera.getLatestResult();

    if (findTarget()){
     
      PhotonTrackedTarget target = result.getBestTarget();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      Translation3d cameraOffset = new Translation3d(-0.2667, 0, 0);
      Rotation3d cameraRotationOffset = new Rotation3d(0, 0, Math.PI);

      Transform3d cameraToRobot = new Transform3d(cameraOffset, cameraRotationOffset);

      Transform3d targetToRobot = cameraToRobot.plus(bestCameraToTarget.inverse());

      return targetToRobot.getX();

    }

    return 0;

  }


  public double getYDistanceToTarget(){
    // gets left/right distance in meters (left is positive)
    var result = camera.getLatestResult();

    if (findTarget()){
     
      PhotonTrackedTarget target = result.getBestTarget();
      Transform3d bestCameraToTarget = target.getBestCameraToTarget();
      Translation3d cameraOffset = new Translation3d(-0.2667, 0, 0);
      Rotation3d cameraRotationOffset = new Rotation3d(0, 0, Math.PI);

      Transform3d cameraToRobot = new Transform3d(cameraOffset, cameraRotationOffset);

      Transform3d targetToRobot = cameraToRobot.plus(bestCameraToTarget.inverse());
      
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
    

      return targetToRobot.getY();

    }

    return 0;

  }


  public double getTargetAngle(double x, double y){

    // gives angle of the target in degrees
    double targetAngle = (Math.tan(y/x)) * (180/Math.PI);
    return targetAngle;

  }


  private double getPitch(){
    // gets pitch of target in radians
    var result = camera.getLatestResult();

    if (findTarget()){
      PhotonTrackedTarget target = result.getBestTarget();

      double pitch = target.getPitch();

      return pitch;

    }
    return 0;
  }

  public double distanceFormula(double x, double y){
    // returns distance to target in meters
    return Math.sqrt((x*x)+(y*y));
  }



  public void updateSmartDash(){

    SmartDashboard.putBoolean("hasTargets",findTarget());
    //SmartDashboard.putNumber("Target ID", getTagID());
    //SmartDashboard.putNumber("Pitch (Deg)", getPitch());
    SmartDashboard.putNumber("Target X Distance", getXDistanceToTarget());
    SmartDashboard.putNumber("Target Y Distance", getYDistanceToTarget());
    //SmartDashboard.putNumber("Target Total Distance", distanceFormula(getXDistanceToTarget(), getYDistanceToTarget()));
    //SmartDashboard.putNumber("Target Angle", getTargetAngle(getXDistanceToTarget(), getYDistanceToTarget()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}