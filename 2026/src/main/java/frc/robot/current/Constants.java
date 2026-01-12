// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.current;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final String robot = "Real";
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOtherControllerPort = 1;

     
  }

  public class PivotConstants {
    public static final double pivotOffset = -5;
    // Pivot angles
    public static final double pivotStoredAngle = 1; //TODO: Get this one. Angle when stored or robot off.
    public static final double pivotReef = 80 ;
    public static final double pivotGround = 84;
    public static final double pivotNetAngle = 140;
    public static final double pivotPartialStoredAngle = 25;
    public static final double pivotSource = 23;
    public static final double pivotStowAway = 170;

    public static final double maxAngle = 175;
    public static final double minAngle = 0;

    public static boolean algaeDown = false; 

    public static final double kP = 0.15;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kS = 0.0;
    public static final double kG = 0.2;
    public static final double kV = 0.1;
    public static final double kA = 0.0;

    // motor constants
    public static final int algaePivotID = 52;
    public static final int algaeFlyLeftID = 50;
    public static final int algaeFlyRightID = 51;

    public static final double algaeConstantThreshold = 1;
}
}
