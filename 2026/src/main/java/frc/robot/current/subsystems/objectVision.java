package frc.robot.current.subsystems;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class objectVision extends SubsystemBase {
    private ArrayList<Integer> cameraIds;

    public objectVision(ArrayList<Integer> camerasIds) {
        this.cameraIds = camerasIds;
    }

    public void getPath() {
        TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0); // m/s, m/s^2
        config.setKinematics(drive.getKinematics());

        Pose2d start = waypoints.get(0);
        Pose2d end = waypoints.get(waypoints.size() - 1);
        List<Translation2d> interiorPoints = new ArrayList<>();
        for (int i = 1; i < waypoints.size() - 1; i++) {
            interiorPoints.add(waypoints.get(i).getTranslation());
        }

        return TrajectoryGenerator.generateTrajectory(
            start, interiorPoints, end, config
        );        
    }

}
