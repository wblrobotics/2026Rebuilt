// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.current;

import static edu.wpi.first.apriltag.AprilTagFields.k2026RebuiltWelded;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import java.io.IOException;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(651.22);
  public static double fieldWidth = Units.inchesToMeters(317.7);

  public static Translation2d ampCenter = new Translation2d(Units.inchesToMeters(72.455),
      Units.inchesToMeters(322.996));

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static double centerlineX = fieldLength / 2.0;

    // need to update
    public static double centerlineFirstY = Units.inchesToMeters(29.638);
    public static double centerlineSeparationY = Units.inchesToMeters(66);
    public static double spikeX = Units.inchesToMeters(114);
    // need to update
    public static double spikeFirstY = Units.inchesToMeters(161.638);
    public static double spikeSeparationY = Units.inchesToMeters(57);

    public static Translation2d[] centerlineTranslations = new Translation2d[5];
    public static Translation2d[] spikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] = new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
    }
  }

  /** The centers of the blue alliance elements */
  public static final class Elements {

    public static Translation2d outpost = new Translation2d(
        0.35,
        0.68);

    public static Translation2d depot = new Translation2d(
      1,
      5.9);

    public static Translation2d hub = new Translation2d(
      3.6,
      4
    );

    public static Translation2d leftTrench = new Translation2d(
      3.6,
      7.3);
      
    public static Translation2d rightTrench = new Translation2d(
      3.6,
      0.7);

    public static Pose2d hubPose = new Pose2d(hub, new Rotation2d(0.0));

  }




  public static double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2026RebuiltWelded.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}