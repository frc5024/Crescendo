// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.UncheckedIOException;
import java.util.Vector;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/** Add your docs here. */
public class VisionModule {
    private static VisionModule mInstance = null;
    private double[] lastTranslation;
    private double[] lastRotation;
    private PhotonCamera frontCamera;
    private AprilTagFieldLayout aprilTagFieldLayout;
    private double neededRotation;

    public VisionModule(AprilTagFields aprilTagFields) {
        try {
            this.aprilTagFieldLayout = aprilTagFields.loadAprilTagLayoutField();
            // aprilTagFieldLayout.setOrigin(this.originPosition);
        } catch (UncheckedIOException e) {
            DriverStation.reportError("Failed to load AprilTag Field Layout", e.getStackTrace());
        }

        frontCamera = new PhotonCamera(Constants.VisionConstants.REAR_CAMERA_NAME);

    }

    public void close() {
        frontCamera.close();
    }

    public void periodic() {

    }

    public double getDistance() {
        var result = frontCamera.getLatestResult();
        if (result.hasTargets()) {
            SmartDashboard.putNumber("Testing", 2);
            var best = result.getBestTarget();
            var transform = best.getBestCameraToTarget();
            var output = transform.getTranslation().getNorm();
            SmartDashboard.putNumber("Best", best.getFiducialId());

            return output;

        }

        return 0;

    }

    public double[] getTranslation(int id) {
        var result = frontCamera.getLatestResult();
        double[] out = { 0, 0, 0 };
        for (var target : result.getTargets()) {
            if (target.getFiducialId() == id) {
                var transform = target.getBestCameraToTarget();

                var output = transform.getTranslation();
                lastTranslation = new double[] { output.getX(), output.getY(), output.getZ() };
                return lastTranslation;
            }

        }

        return out;
    }

    public double getRotation(int id) {
        var result = frontCamera.getLatestResult();
        for (var target : result.getTargets()) {
            if (target.getFiducialId() == id) {
                var transform = target.getBestCameraToTarget();

                var output = transform.getRotation();
                neededRotation = output.getZ();
                return neededRotation;
            }
        }

        return 0;
    }

    // public boolean shouldShoot() {
    // var result = frontCamera.getLatestResult();
    // if (result.hasTargets()) {
    // var best = result.getBestTarget();
    // var transform = best.getBestCameraToTarget();
    // var id = Robot.visionModule.getID();
    // if (id == Constants.VisionConstants.SPEAKER_ID) { //changed name
    // var X = transform.getX();
    // if (X > 1 && X < 2) {
    // return true;

    // }

    // }

    // }
    // return false;
    // }

    public boolean hasTarget() {
        SmartDashboard.putBoolean("HasTarget", true);
        SmartDashboard.putString("Name", frontCamera.getName());
        var K = frontCamera.getCameraMatrix();
        if (K.isPresent()) {
            SmartDashboard.putNumberArray("K", K.get().getData());
        } else {
            SmartDashboard.putNumberArray("KK", new double[] { 1, 2, 3 });
        }
        SmartDashboard.putBoolean("Is connected", frontCamera.isConnected());
        // frontCamera.takeInputSnapshot();
        try {
            var result = frontCamera.getLatestResult();
            return result.hasTargets();
        } catch (Exception e) {
            e.printStackTrace();
            throw e;
        }
    }

    public Vector<Integer> getID() {
        var result = frontCamera.getLatestResult();
        Vector<Integer> tags = new Vector<>();
        if (result.hasTargets()) {
            var best = result.getTargets();
            for (PhotonTrackedTarget tag : best) {
                tags.addElement(tag.getFiducialId());
            }
        }
        // System.out.println(tags);
        return tags;
    }

}