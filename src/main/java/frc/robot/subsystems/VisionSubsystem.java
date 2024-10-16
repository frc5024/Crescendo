// package frc.robot.subsystems;

// import java.util.ArrayList;
// import java.util.List;
// import java.util.function.Supplier;

// import org.photonvision.EstimatedRobotPose;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.camera.Camera;
// import frc.robot.modules.vision.VisionModule;

// /**
// *
// */
// public class VisionSubsystem extends SubsystemBase {
// /* Modules */
// private List<VisionModule> visionModules = new ArrayList<VisionModule>();

// /**
// *
// */
// public VisionSubsystem(List<Camera> cameras) {
// for (Camera camera : cameras) {
// this.visionModules.add(new VisionModule(camera));
// }
// }

// @Override
// public void periodic() {
// try {
// for (VisionModule visionModule : this.visionModules) {
// visionModule.process();
// }
// } catch (Exception e) {
// System.out.print(e.getMessage());
// }
// }

// /**
// * Use AprilTag camera pose estimation
// */
// public List<EstimatedRobotPose> getBestLatestEstimatedPoses() {
// List<EstimatedRobotPose> poses = new ArrayList<EstimatedRobotPose>();

// for (VisionModule visionModule : this.visionModules) {
// if (visionModule.getCameraType() != Camera.Type.APRILTAG)
// continue;

// EstimatedRobotPose pose = visionModule.getBestLatestEstimatedPose();
// if (pose == null)
// continue;

// poses.add(pose);
// }

// return poses;
// }

// /**
// * Use target from the first Game Piece camera
// */
// public PhotonTrackedTarget getBestNoteTarget() {
// VisionModule visionModule =
// getVisionModuleByType(Camera.Type.COLOURED_SHAPE);

// return visionModule == null ? null : visionModule.getBestTarget();
// }

// /**
// *
// */
// public PhotonTrackedTarget getBestTarget(String cameraName) {
// VisionModule visionModule = getVisionModuleByName(cameraName);

// return visionModule == null ? null : visionModule.getBestTarget();
// }

// /**
// *
// */
// public double getCameraYaw(String cameraName) {
// VisionModule visionModule = getVisionModuleByName(cameraName);

// return visionModule == null ? null : visionModule.getCameraYaw();
// }

// /**
// *
// */
// public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose) {
// VisionModule visionModule = getVisionModuleByType(Camera.Type.APRILTAG);

// return visionModule == null ? null :
// visionModule.getEstimationStdDevs(estimatedPose);
// }

// /**
// *
// */
// public Transform3d getRobotToCamera(String cameraName) {
// VisionModule visionModule = getVisionModuleByName(cameraName);

// return visionModule == null ? null : visionModule.getCameraRobotToCamera();
// }

// /**
// *
// */
// public VisionModule getVisionModuleByName(String cameraName) {
// VisionModule visionModule = this.visionModules.stream()
// .filter(module -> module.getCameraName() == cameraName)
// .findFirst()
// .get();

// return visionModule;
// }

// /**
// *
// */
// private VisionModule getVisionModuleByType(Camera.Type type) {
// VisionModule visionModule = this.visionModules.stream()
// .filter(module -> module.getCameraType() == type)
// .findFirst()
// .get();

// return visionModule;
// }

// /**
// * Only used in simulation
// */
// public void resetFieldPosition(Pose2d pose2d) {
// for (VisionModule visionModule : this.visionModules) {
// visionModule.resetFieldPosition(pose2d);
// }
// }

// /**
// * Only used in simulation
// */
// public void setPoseSupplier(Supplier<Pose2d> poseSupplier) {
// for (VisionModule visionModule : this.visionModules) {
// visionModule.setPoseSupplier(poseSupplier);
// }
// }
// }
