// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

  //Replace cameraName with the name of the camera on the robot
  private PhotonCamera camera;
  private NetworkTableInstance networkTableInstance;
  private double aprilTagX;
  private double aprilTagY;
  
  public VisionSubsystem() {
    camera = new PhotonCamera("cameraName");
    networkTableInstance = NetworkTableInstance.getDefault();
  }

  @Override
  public void periodic() {
    PhotonPipelineResult result = camera.getLatestResult();
    if(result.hasTargets()) {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      aprilTagX = bestTarget.getYaw();
      aprilTagY = bestTarget.getPitch();

      //Change to the name of the network table uploaded to PhotonVision
      networkTableInstance.getTable("apriltag").getEntry("x").setDouble(aprilTagX);
      networkTableInstance.getTable("apriltag").getEntry("y").setDouble(aprilTagY);
    }
  }

  public double getAprilTagX() {
    return aprilTagX;
  }
  public double getAprilTagY() {
    return aprilTagY;
  }
}