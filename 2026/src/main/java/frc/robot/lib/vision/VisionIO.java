package frc.robot.lib.vision;

import java.util.Optional;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.*;

public interface VisionIO {
    //@AutoLog
    public static class VisionIOInputs {
        public int tagCount = 0;
        public Pose3d estimatedPose = new Pose3d();
        public double timestamp = 0.0;
        public Matrix<N3, N1> stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        
        public Pose2d robotPose = new Pose2d();
    }

    public void updateInputs(VisionIOInputs inputs);
}
