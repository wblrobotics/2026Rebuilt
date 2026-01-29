package frc.robot.lib.vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.current.Constants;
import frc.robot.current.Robot;

public class VisionIOPhotonVision implements VisionIO {

    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private double lastEstTimestamp = 0;
    
    // Simulation
    private PhotonCameraSim cameraSim;
    private VisionSystemSim visionSim;

    private AprilTagFieldLayout aprilTagField;

    private boolean detectsFuel;

    /**
     * 
     * @param cameraName The name of the camera found in photonvision
     * @param cameraPose3d The location of the robot relative to it's center
     */

    public VisionIOPhotonVision(String cameraName, Pose3d cameraPose3d) {

        switch (Constants.fieldType) {
            case "Welded": 
                aprilTagField = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
            break;
            case "AndyMark":
                aprilTagField = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
            break;
        }

        System.out.println("[Init] Creating VisionIOPhotonVision Camera: " + cameraName);
        camera = new PhotonCamera(cameraName);

        photonEstimator = 
            new PhotonPoseEstimator(aprilTagField, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Pose3d(), cameraPose3d));
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Simulation
        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(aprilTagField);

            // Simulated Camera Properties
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(70));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(35);
            cameraProp.setAvgLatencyMs(40);
            cameraProp.setLatencyStdDevMs(5);

            // Create PhotonCameraSim to update the linked PhotonCamera with visible targets
            cameraSim = new PhotonCameraSim(camera, cameraProp);

            // Add the simulated camera to view the targets on the simulated field
            visionSim.addCamera(cameraSim, new Transform3d(new Pose3d(), cameraPose3d));

            cameraSim.enableDrawWireframe(true);
        }
    }

    public PhotonPipelineResult getLatestResult() {
        return camera.getLatestResult();
    }

    /** 
     * The latest estimated robot pose on the field based on this vision data. This may be empty.
     * 
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        var visionEst = photonEstimator.update(getLatestResult());
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;
        if (Robot.isSimulation()) {
            visionEst.ifPresentOrElse(
                est ->
                    getSimDebugField()
                        .getObject("VisionEstimation")
                        .setPose(est.estimatedPose.toPose2d()),
                () -> {
                    if (newResult) getSimDebugField().getObject("VisionEstimation").setPoses();
                });
        }
        if (newResult) lastEstTimestamp = latestTimestamp;
        return visionEst;
    }

    /**
     * The standard deviations of the estimated pose from {@link #getEstimatedGlobalPose()}, for use
     * with a PoseEstimator.
     * This should only be used when there are targets visible.
     * 
     * @param estimatedPose The estimated pose to guess standard deviations for.
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose3d estimatedPose) {
        Pose2d estimatedPose2d = estimatedPose.toPose2d();
        var estStdDevs = VecBuilder.fill(4, 4, 8); // Generic StdDevs for single tag usage
        var targets = getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist += tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose2d.getTranslation());
        }
        if (numTags == 0) return estStdDevs; // Return default values if no tags are visible
        avgDist /= numTags;
        // Decrease StdDevs if multiple targets are visible
        if (numTags > 1) estStdDevs = VecBuilder.fill(0.5, 0.5, 1); // Generic StdDevs for multitag
        // Increase StdDevs based on (average) distance
        if (numTags == 1 && avgDist > 3) // Original value = 4
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE); // If single tag + 4 meters away we probably shouldnt be using this pose data
        else if (avgDist > 6)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30)); // Original value = 30. Reduced to trust further measurements less

        // Additional filtering
        // Throw out pose if off the ground
        if (estimatedPose.getZ() > 0.25 || estimatedPose.getZ() < -0.25) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        return estStdDevs;
    }

    // Simulation code
    
    public void simulationPeriodic(Pose2d robotSimPose) {
        visionSim.update(robotSimPose);
    }

    public void resetSimPose(Pose2d pose) {
        if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
    }

    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        if (Robot.isSimulation()) {
            visionSim.update(inputs.robotPose);
        }

        inputs.tagCount = getLatestResult().getTargets().size();
        var poseEst = getEstimatedGlobalPose();
        poseEst.ifPresentOrElse(
            est -> {
                inputs.estimatedPose = est.estimatedPose;
                inputs.stdDevs = getEstimationStdDevs(est.estimatedPose);
                //inputs.timestamp = Units.secondsToMilliseconds(est.timestampSeconds);
                inputs.timestamp = est.timestampSeconds;
            },
            () -> {
                inputs.estimatedPose = null;
                inputs.stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            });
    }
}
