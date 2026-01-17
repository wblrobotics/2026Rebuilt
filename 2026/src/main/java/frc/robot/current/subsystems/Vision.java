package frc.robot.current.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.current.Constants;
import frc.robot.lib.vision.VisionIO;
import frc.robot.lib.vision.VisionIO.VisionIOInputs;
import frc.robot.lib.util.PoseEstimator.TimestampedVisionUpdate;

public class Vision extends SubsystemBase {
    private static final Pose3d[] cameraPoses;
    private static final double xyStdDevCoefficient;
    private static final double thetaStdDevCoefficient;

    private final VisionIO[] io;
    private final VisionIOInputs[] inputs;

    private boolean enableVisionUpdates = true;
    private Consumer<List<TimestampedVisionUpdate>> visionConsumer = (x) -> {};
    private Supplier<Pose2d> poseSupplier = () -> new Pose2d();
    private Map<Integer, Double> lastFrameTimes = new HashMap<>();
    //private Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    static {
        switch (Constants.robot) {
            case "SIM":
                cameraPoses = 
                    new Pose3d[] {
                        // Camera mounted on front of robot
                        new Pose3d(
                            Units.inchesToMeters(13), // 13 inches forward
                            Units.inchesToMeters(0), // 0 inches to the left
                            Units.inchesToMeters(3.75), // 3.75 inches off the ground
                            new Rotation3d() // no rotation for now
                        )
                    };
                xyStdDevCoefficient = 0.01;
                thetaStdDevCoefficient = 0.01;
                break;
            case "Real":
                cameraPoses = 
                    new Pose3d[] {
                        // Camera mounted on front of robot
                        new Pose3d(
                            Units.inchesToMeters(13), // 13 inches forward
                            Units.inchesToMeters(0), // 0 inches to the left
                            Units.inchesToMeters(3.75), // 3.75 inches off the ground
                            new Rotation3d() // no rotation for now
                        )
                    };
                xyStdDevCoefficient = 0.01;
                thetaStdDevCoefficient = 0.01;
                break;
            default:
                cameraPoses = new Pose3d[] {};
                xyStdDevCoefficient = 0.01;
                thetaStdDevCoefficient = 0.01;
                break;
        }
    }

    public Vision(VisionIO... io) {
        System.out.println("[Init] Creating Vision");
        this.io = io;
        inputs = new VisionIOInputs[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new VisionIOInputs();
        }

        // Multi Cam
        // Create map of last frame times for instances
        for (int i = 0; i < io.length; i++) {
            lastFrameTimes.put(i, 0.0);
        }

    }

    public void setDataInterfaces(Consumer<List<TimestampedVisionUpdate>> visionConsumer, Supplier<Pose2d> poseSupplier) {
        this.visionConsumer = visionConsumer;
        this.poseSupplier = poseSupplier;
    }

    /** Sets whether vision updates for odometry are enabled. */
    public void setVisionUpdatesEnabled(boolean enabled) {
        enableVisionUpdates = enabled;
    }

    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            //Logger.processInputs("Vision/Inst" + Integer.toString(i), inputs[i]);
        }

        // Loop instances
        // List<Pose2d> allRobotPoses = new ArrayList<>();
        // List<Pose3d> allRobotPoses3d = new ArrayList<>();
        List<TimestampedVisionUpdate> visionUpdates = new ArrayList<>();
        for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
            // Give the PhotonVision instance a new robot pose for sim purposes...
            inputs[instanceIndex].robotPose = poseSupplier.get();
            
            // If no tags were found we can move onto the next instance
            if (inputs[instanceIndex].tagCount == 0) {
                continue;
            }

            // If the pose estimate is null, we also can move on
            if (inputs[instanceIndex].estimatedPose == null) {
                continue;
            }

            // Now for the fun stuff
            // Add vision update to pose estimator
            visionUpdates.add(new TimestampedVisionUpdate(inputs[instanceIndex].timestamp, inputs[instanceIndex].estimatedPose.toPose2d(), inputs[instanceIndex].stdDevs));

            // Log instance data with AdvantageKit
            Logger.recordOutput("Vision/Inst" + Integer.toString(instanceIndex) + "/RobotPose", inputs[instanceIndex].estimatedPose.toPose2d());
            Logger.recordOutput("Vision/Inst" + Integer.toString(instanceIndex) + "/RobotPose3d", inputs[instanceIndex].estimatedPose);
            Logger.recordOutput("Vision/Inst" + Integer.toString(instanceIndex) + "/StdDevs", inputs[instanceIndex].stdDevs.getData());
            Logger.recordOutput("VisionEnabled", enableVisionUpdates);
        }

        if (enableVisionUpdates) {
            visionConsumer.accept(visionUpdates);
        }
    }
}
