// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Includes camera setup and all methods for getting camera data */
public class Vision extends SubsystemBase{

    public static PhotonCamera centerCamera = new PhotonCamera("Center");
  

    public final static AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();
    
  

    private final static Transform3d robotToCenterCamera = new Transform3d(
        new Translation3d(0, 0, 0),
        new Rotation3d(0, 0,0)
    );
    

    public final static PhotonPoseEstimator centerCameraPoseEsimator = new PhotonPoseEstimator(
    aprilTagFieldLayout, 
    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
    robotToCenterCamera);

    public final static PhotonPipelineResult centerCameraPipelineResult = new PhotonPipelineResult();

    private static double centerCameraEstimatedYaw = 0;
    private static double centerCameraEstimatedPitch = 0;

    public static double[] centerCameraPose = {0,0,0,0};

    public static double[] previousAmpCameraPose = {0,0,0,0};

    public static Pose3d centerPose3d = new Pose3d();
    public static Pose2d centerPose2d = new Pose2d();

    public static double centerTimeStamp = 0;



    public Vision() {
 
    }

    public static boolean isTarget(PhotonCamera camera) {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Returns an array with all targets currently detected by the camera
     * 
     * @param camera camera to use
     * @return Object[] list of all targets
     */
    public static Object[] getListOfTargets(PhotonCamera camera) {
        return camera.getLatestResult().getTargets().toArray();
    }

    /**
     * Used with a for loop to check if a desired tag ID is included in the list of targets
     * 
     * @param camera camera to use
     * @param i array index to check
     * @return tagID at that array index
     */
    public static int getTagIDFromList(PhotonCamera camera, int i) {

        return camera.getLatestResult().getTargets().get(i).getFiducialId();

    }

    /**
     * Returns yaw angle of the target in the camera space
     * 
     * @param camera camera to use
     * @param targetTagID desired AprilTag
     * @return yaw angle of target in degrees
     */
    public static double getYaw(PhotonCamera camera, int targetTagID) {
        double yaw = 180.0;
        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    yaw = targets.get(i).getYaw();

                }

            }

        }
        switch (camera.getName()) {
          case "Center":
             centerCameraEstimatedYaw = yaw;
            break;
          default:
            System.out.println("This shouldn't print.");
            break;
        }
        return yaw;
       
    }

    /**
     * Returns pitch angle of the target in the camera space
     * 
     * @param camera camera to use
     * @param targetTagID desired AprilTag
     * @return pitch angle of target in degrees
     */
    public static double getPitch(PhotonCamera camera, int targetTagID) {
        double pitch = 180.0;
        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    pitch = targets.get(i).getPitch();

                }

            }

        }
        switch (camera.getName()) {
          case "Camera6":
            centerCameraEstimatedPitch = pitch;
            break;
          default:
            System.out.println("This shouldn't print.");
            break;
        }
        return pitch;

    }

    /**
     * Returns the area of the camera field of view that the target takes up
     * 
     * @param camera camera to use
     * @param targetTagID desired AprilTag
     * @return area of target as a percent from 0-100
     */
    public static double getArea(PhotonCamera camera, int targetTagID) {

        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    return targets.get(i).getArea();

                }

            }

        } else {

            return 0.0;

        }

        return 0.0;

    }


    /**
     * Returns 
     * 
     * @param camera
     * @return
     */
    public static double getTagAmbiguity(PhotonCamera camera, int targetTagID) {
        var result = camera.getLatestResult();

        if(result.hasTargets()) {

            var targets = result.getTargets();

            for(int i = 0; i < targets.size(); i++) {

                if(targets.get(i).getFiducialId() == targetTagID) {

                    return targets.get(i).getPoseAmbiguity();

                }

            }

        } else {

            return 1.0;

        }

        return 1.0;

    }




}