// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.team1787_lib.ModuleConfiguration;

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

  public static final class Swerve {
    /**
     * distance between the center point of the left wheels and the center point of
     * the right wheels
     */
    public static final double trackwidthMeters = Units.inchesToMeters(20.75);
    /**
     * distance between the center point of the front wheels and the center point of
     * the back wheels
     */
    public static final double wheelbaseMeters = Units.inchesToMeters(20.75);

    public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double wheelDiamater = 0.10033;

    public static final ModuleConfiguration MK4I_L2 = new ModuleConfiguration(
        Units.inchesToMeters(4), // 0.10033,
        (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0),
        true,
        (14.0 / 50.0) * (10.0 / 60.0),
        false);
    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
        new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
        new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
        new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0));

    // pid for drive motor
    public static final double drivekP = 0.8; // 0.049867
    public static final double drivekI = 0.;
    public static final double drivekD = 0.01;

    // pid for angle motor
    public static final double anglekP = 0.065;
    public static final double anglekI = 0.;
    public static final double anglekD = 0.; // 0.00006

    // feedforward for drive motor (CHARACTERIZE TO FIND)
    public static final double drivekS = 0.12584;
    public static final double drivekV = 2.6; // .8679
    public static final double drivekA = 0.14785;

    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // 4.578870701248506
    // TODO: fix this later
    public static final double maxVelocityMetersPerSecond = 5880.0 / 60.0 *
        driveReduction *
        wheelDiamater * Math.PI;

    /**
     * The maximum angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place.
     */
    // Here we calculate the theoretical maximum angular velocity. You can also
    // replace this with a measured amount.
    // theoretical - 12.286312581459901
    public static final double maxAngularVelocityRadiansPerSecond = maxVelocityMetersPerSecond /
        Math.hypot(Constants.Swerve.trackwidthMeters / 2.0, Constants.Swerve.wheelbaseMeters / 2.0);

    public static final int angleContinuousCurrentLimit = 0;
    public static final boolean angleInvert = true;
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final int driveContinuousCurrentLimit = 0;
    public static final boolean driveInvert = true;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    public static final class CANCoderOffsets {
      public static final double one = 99.74;
      public static final double two = 287.23;
      public static final double three = 211.99;
      public static final double four = 0.00;// 254.17969796160835;
      public static final double five = 107.00;// 297.68554453125;
      public static final double six = 354.73;// 84.73;
      public static final double seven = 0.00;
      public static final double eight = 0.00;
      public static final double nine = 0.00;
    }

    public enum SwerveModuleCorners {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }

    public static double getOffset(double leftRightOffset, SwerveModuleCorners corner) {
      switch (corner) {
        case FRONT_LEFT:
          return leftRightOffset;
        case FRONT_RIGHT:
          return leftRightOffset - 90;
        case BACK_LEFT:
          return leftRightOffset + 90;
        case BACK_RIGHT:
          return leftRightOffset + 180;
        default:
          return leftRightOffset;
      }
    }

    public static final class FrontLeftSwerveModule {
      /** CAN ID for the SPARK MAX used to control the front left driving NEO */
      public static final int driveMotorID = 2;
      /** CAN ID for the SPARK MAX used to control the front left steering NEO */
      public static final int steerMotorID = 1;
      /** CAN ID for the front left CANcoder */
      public static final int steerEncoderID = 5;
      /** Offset from true zero for the front left swerve module in degrees */
      public static final double steerOffset = Constants.Swerve.getOffset(
          CANCoderOffsets.five, SwerveModuleCorners.FRONT_LEFT);// 107;
    }

    public static final class FrontRightSwerveModule {
      /** CAN ID for the SPARK MAX used to control the front right driving NEO */
      public static final int driveMotorID = 4;
      /** CAN ID for the SPARK MAX used to control the front right steering NEO */
      public static final int steerMotorID = 3;
      /** CAN ID for the front right CANcoder */
      public static final int steerEncoderID = 3;
      /** Offset from true zero for the front right swerve module in degrees */
      public static final double steerOffset = Constants.Swerve.getOffset(
          CANCoderOffsets.three,
          SwerveModuleCorners.FRONT_RIGHT);// 121.99;
    }

    public static final class BackLeftSwerveModule {
      /** CAN ID for the SPARK MAX used to control the back left driving NEO */
      public static final int driveMotorID = 8;
      /** CAN ID for the SPARK MAX used to control the back left steering NEO */
      public static final int steerMotorID = 7;
      /** CAN ID for the back left CANcoder */
      public static final int steerEncoderID = 6;
      /** Offset from true zero for the back left swerve module in degrees */
      public static final double steerOffset = Constants.Swerve.getOffset(
          CANCoderOffsets.six,
          SwerveModuleCorners.BACK_RIGHT);// -185.27;
    }

    public static final class BackRightSwerveModule {
      /** CAN ID for the SPARK MAX used to control the back right driving NEO */
      public static final int driveMotorID = 6;
      /** CAN ID for the SPARK MAX used to control the back right steering NEO */
      public static final int steerMotorID = 5;
      /** CAN ID for the back right CANcoder */
      public static final int steerEncoderID = 1;
      /** Offset from true zero for the back right swerve module in degrees */
      public static final double steerOffset = Constants.Swerve.getOffset(
          CANCoderOffsets.one,
          SwerveModuleCorners.BACK_RIGHT);// 279.73;
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class IntakeIndexerConstants {
    public static int intakeOutPneumaticsChannel = 0;
    public static int intakeInPneumaticsChannel = 1;

    public static int intakeMotorID = 98;
    public static int conveyorMotorID = 99;


    public static double intakeMotorSpeed = .5;
    public static double conveyorMotorSpeed = .5;
  }

  public static class ElevatorGrabberConstants {
    public static int elevatorMotorID = 99;
    public static int grabberMotorID = 99;

    public static int outPneumaticChannel = 99;
    public static int inPneumaticChannel = 99;

    public static int boreEncoderID = 99;
    public static int distancePerRotation = 99;

    public static int upperLimitSwitchID = 99;
    public static int lowerLimitSwitchID = 99;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double kS = 0;
    public static double kV = 0;
  }

  public static class LEDConstants {
    public static int BlinkinRedColor = 0;
    public static int BlinkinBlueColor = 0;
    public static int BlinkinYellowColor = 0;
    public static int BlinkinPurpleColor = 0;
  }

  public static class ControllerConstants {
    public static final double controllerDeadzone = 0.175;
  }

  public static class Vision {
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