// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.UncheckedIOException;

import org.photonvision.PhotonCamera;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;

/** Add your docs here. */
public class VisionModule {
    private double[] lastTranslation;
    private double[] lastRotation;
    private PhotonCamera frontCamera;
    private AprilTagFieldLayout aprilTagFieldLayout;

    public VisionModule(AprilTagFields aprilTagFields) {
        try {
            this.aprilTagFieldLayout = aprilTagFields.loadAprilTagLayoutField();
            // aprilTagFieldLayout.setOrigin(this.originPosition);
        } catch (UncheckedIOException e) {
            DriverStation.reportError("Failed to load AprilTag Field Layout", e.getStackTrace());
        }

        frontCamera = new PhotonCamera(Constants.VisionConstants.FRONT_CAMERA_NAME);

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

    public double[] getTranslation() {
        var result = frontCamera.getLatestResult();
        if (result.hasTargets()) {
            var best = result.getBestTarget();
            var transform = best.getBestCameraToTarget();
            Translation3d output = transform.getTranslation();
            lastTranslation = new double[] { output.getX(), output.getY(), output.getZ() };

        }

        return lastTranslation;
    }

    public double[] getRotation() {
        var result = frontCamera.getLatestResult();
        if (result.hasTargets()) {
            var best = result.getBestTarget();
            var transform = best.getBestCameraToTarget();
            var output = transform.getRotation();
            lastRotation = new double[] { (output.getX()), (output.getY()), (output.getZ()) };

        }

        return lastRotation;
    }

    public boolean shouldShoot() {
        var result = frontCamera.getLatestResult();
        if (result.hasTargets()) {
            var best = result.getBestTarget();
            var transform = best.getBestCameraToTarget();
            var id = Robot.visionModule.getID();
            if (id == Constants.VisionConstants.SPEAKER_ID) {
                var X = transform.getX();
                if (X > 1 && X < 2) {
                    return true;

                }

            }

        }
        return false;
    }

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

    public int getID() {
        var result = frontCamera.getLatestResult();
        if (result.hasTargets()) {
            var best = result.getBestTarget();
            var translation = best.getBestCameraToTarget();
            var output = best.getFiducialId();

            return output;

        }
        return -1;
    }

}
