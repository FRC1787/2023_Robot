// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class GoofyAhhConstants {
    public static final boolean weWillWin = true;
    public static final boolean weWillLose = false;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeIndexerConstants {
    public static int intakeOutPneumaticsChannel = 0;
    public static int intakeInPneumaticsChannel = 1;

    public static int intakeMotorID = 99;
    public static double intakeMotorSpeed = .5;
  }

  public static class LEDConstants {
    public static int BlinkinRedColor = 0;
    public static int BlinkinBlueColor = 0;
    public static int BlinkinYellowColor = 0;
    public static int BlinkinPurpleColor = 0;
  }

  public static class LimelightConstants {
    public static enum LimelightTarget {
      aprilTag(0, 0.46),
      midTape(1, 0.61),
      highTape(2, 1.11);
      
      public int limelightPipeline;
      public double heightMeters;
      LimelightTarget(int limelightPipeline, double heightMeters) {
        this.limelightPipeline=limelightPipeline;
        this.heightMeters=heightMeters;
      }
    }

    /** Height of the Limelight from the ground. */
    public static final double limelightHeightMeters = 0.432;
    /** Angle above the horizontal of the Limelight. */
    public static final double limelightAngleDegrees = 14.61;

    public static final double limelightLateralOffsetMeters = 0.152;
  }
}