// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
    // KINEMATICS CONSTANTS
    /**
     * distance between the center point of the left wheels and the center point of
     * the right wheels
     */
    public static final double trackwidthMeters = Units.inchesToMeters(22.75);
    /**
     * distance between the center point of the front wheels and the center point of
     * the back wheels
     */
    public static final double wheelbaseMeters = Units.inchesToMeters(22.75);
    public static final double driveReduction = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double steerReduction = (14.0 / 50.0) * (10.0 / 60.0);

    public static final double wheelDiamaterMeters = Units.inchesToMeters(4.0);
    public static final double wheelCircumferenceMeters = wheelDiamaterMeters * Math.PI;

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
        new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
        new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
        new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0));


    // PID + FEEDFORWARD CONSTANTS FOR MOTORS
    // pid for drive motor
    public static final double drivekPVoltsPerMeterPerSecond = 0.8;
    public static final double drivekIVoltsPerMeterPerSecondSquared = 0.;
    public static final double drivekDVoltsPerMeter = 0.01;

    // pid for angle motor
    public static final double anglekPVoltsPerDegree = 0.065;
    public static final double anglekIVolts = 0.; // this might be the wrong unit idk 
    public static final double anglekDVoltsPerDegreePerSecond = 0.;

    public static final double drivekSVolts = 0.12584;
    public static final double drivekVVoltsSecondsPerMeter = 2.6; // .8679
    public static final double drivekAVoltsSecondsSquaredPerMeter = 0.14785;

    /**
     * The maximum possible velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot will be able to drive in a straight
     * line, based off of the empirical free speed of the drive NEOs.
     */
    public static final double maxAchievableVelocityMetersPerSecond = 5880.0 / 60.0 *
        driveReduction *
        wheelDiamaterMeters * Math.PI;

    /**
     * This is the max desired speed that will be achievable in teleop.
     * <p>
     * If the controller joystick is maxed in one direction, it will drive at this speed.
     * <p>
     * This value will be less than or equal to the maxAchievableVelocityMetersPerSecond, depending on driver preference.
     */
    public static final double maxDesiredTeleopVelocityMetersPerSecond = 4.0;

    /**
     * The maximum achievable angular velocity of the robot in radians per second.
     * <p>
     * This is a measure of how fast the robot can rotate in place, based off of maxAchievableVelocityMetersPerSecond.
     */
    public static final double maxAchievableAngularVelocityRadiansPerSecond = maxAchievableVelocityMetersPerSecond /
        Math.hypot(Constants.Swerve.trackwidthMeters / 2.0, Constants.Swerve.wheelbaseMeters / 2.0);

    /**
     * This is the max desired angular velocity that will be achievable in teleop.
     * <p>
     * If the controller rotation joystick is maxed in one direction, it will rotate at this speed.
     * <p>
     * This value will be tuned based off of driver preference.
     */
    public static final double maxDesiredAngularVelocityRadiansPerSecond = 6.0;

    public static final int angleContinuousCurrentLimit = 0;
    public static final boolean angleInvert = true;
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final int driveContinuousCurrentLimit = 0;
    public static final boolean driveInvert = true;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /**
     * Offset for the angle CANCoder encoder.
     * This offsets the angle encoder so that it reads as 0 when pointing forwards.
     * All values are relative to the front left corner configuration.
     */
    public static final class CANCoderOffsets {
      public static final double one = 61.523;
      public static final double two = 314.209;
      public static final double three = 176.045;
      public static final double four = 63.193;// 254.17969796160835;
      public static final double five = 0;// 297.68554453125;
      public static final double six = 0;// 84.73;
      public static final double seven = 0.00;
      public static final double eight = 0.00;
      public static final double nine = 0.00;
    }

    // This enum is used to determine the offset for the swerve modules.
    public enum SwerveModuleCorners {
      FRONT_LEFT,
      FRONT_RIGHT,
      BACK_LEFT,
      BACK_RIGHT
    }

    /**
     * This method is used to get the offset for the swerve modules.
     * 
     * @param leftRightOffset the offset for the left and right swerve modules.
     * This should be a {@link CANCoderOffsets} constant.
     * @param corner the corner of the swerve module to get the offset for.
     * @return the angular offset for the swerve module in degrees.
     */
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
      public static final int driveMotorID = 1;
      /** CAN ID for the SPARK MAX used to control the front left steering NEO */
      public static final int steerMotorID = 2;
      /** CAN ID for the front left CANcoder */
      public static final int steerEncoderID = 1;
      /** Offset from true zero for the front left swerve module in degrees */
      public static final double steerOffset = Constants.Swerve.getOffset(
          CANCoderOffsets.one, SwerveModuleCorners.FRONT_LEFT);// 107;
    }

    public static final class FrontRightSwerveModule {
      /** CAN ID for the SPARK MAX used to control the front right driving NEO */
      public static final int driveMotorID = 3;
      /** CAN ID for the SPARK MAX used to control the front right steering NEO */
      public static final int steerMotorID = 4;
      /** CAN ID for the front right CANcoder */
      public static final int steerEncoderID = 2;
      /** Offset from true zero for the front right swerve module in degrees */
      public static final double steerOffset = Constants.Swerve.getOffset(
          CANCoderOffsets.two,
          SwerveModuleCorners.FRONT_RIGHT);// 121.99;
    }

    public static final class BackLeftSwerveModule {
      /** CAN ID for the SPARK MAX used to control the back left driving NEO */
      public static final int driveMotorID = 5;
      /** CAN ID for the SPARK MAX used to control the back left steering NEO */
      public static final int steerMotorID = 6;
      /** CAN ID for the back left CANcoder */
      public static final int steerEncoderID = 3;
      /** Offset from true zero for the back left swerve module in degrees */
      public static final double steerOffset = Constants.Swerve.getOffset(
          CANCoderOffsets.three,
          SwerveModuleCorners.BACK_RIGHT);// -185.27;
    }

    public static final class BackRightSwerveModule {
      /** CAN ID for the SPARK MAX used to control the back right driving NEO */
      public static final int driveMotorID = 7;
      /** CAN ID for the SPARK MAX used to control the back right steering NEO */
      public static final int steerMotorID = 8;
      /** CAN ID for the back right CANcoder */
      public static final int steerEncoderID = 4;
      /** Offset from true zero for the back right swerve module in degrees */
      public static final double steerOffset = Constants.Swerve.getOffset(
          CANCoderOffsets.four,
          SwerveModuleCorners.BACK_RIGHT);// 279.73;
    }
  }

  public static class IntakeIndexer {
    // for the intake, not for the actual pneumatics
    public static int intakeExtendPneumaticsChannel = 5;
    public static int intakeRetractPneumaticsChannel = 4;

    public static int indexerRetractPneumaticsChannel = 1;
    public static int indexerExtendPneumaticsChannel = 0;

    public static int clawLimitSwitchID;

    public static int intakeMotorID = 10;
    public static int conveyorMotorID = 9;
    public static int clawMotorID = 15;
    public static int leftIndexerMotorID = 11;
    public static int rightIndexerMotorID = 12;

    public static enum IndexerState {
      cone, cube
    }
  }

  public static class ElevatorGrabber {
    public static int elevatorMotorID = 13;
    public static int grabberMotorID = 14;

    public static int elevatorExtendPneumaticChannel = 3;
    public static int elevatorRetractPneumaticChannel = 2;

    public static int distancePerRotation = 99;

    public static int lowerLimitSwitchID = 99;

    public static double kPVoltsPerMeterPerSecond = 0;
    public static double kIVoltsPerMeter = 0;
    public static double kDVoltsPerMeterPerSecondSquared = 0;

    public static double kSVolts = 0.00;
    public static double kVVoltSecondsPerMeter = 0; //represents voltage given to motor per m/s of desired elevator carriage velocity

    public static double elevatorMaxVelMetersPerSecond = 4;
    public static double elevatorMaxAccelMetersPerSecondSquared = 2;

    public static double chainMetersPerRotation = 1.78*Math.PI;
    public static double grabberMetersPerChain = 2;
    public static double grabberMetersPerRotation = chainMetersPerRotation*grabberMetersPerChain;
  }

  public static class LED {
    public static int BlinkinRedColor = 0;
    public static int BlinkinBlueColor = 0;
    public static int BlinkinYellowColor = 0;
    public static int BlinkinPurpleColor = 0;
  }

  public static class Controller {
    public static final double controllerDeadzone = 0.175;

    public static int buttonA = 1;
    public static int buttonB = 2;
    public static int buttonX = 3;
    public static int buttonY = 4;
    public static int leftBumper = 5;
    public static int rightBumper = 6;
    public static int viewButton = 7;
    public static int menuButton = 8;
    public static int leftJoyStickPress = 9;
    public static int rightJoyStickPress = 10;  
  }

  public static class Vision {
    public static enum LimelightTarget {
      aprilTag(0, 0.46, 0.36),
      midTape(1, 0.61, 0.59),
      highTape(2, 1.11, 1.01);
      
      public int limelightPipeline;
      public double heightMeters;
      public double distanceMeters;

      LimelightTarget(int limelightPipeline, double heightMeters, double distanceMeters) {
        this.limelightPipeline=limelightPipeline;
        this.heightMeters=heightMeters;
        this.distanceMeters = distanceMeters;
      }
    }

    /** Height of the Limelight from the ground. */
    public static final double limelightHeightMeters = 0.432;
    /** Angle above the horizontal of the Limelight. */
    public static final double limelightAngleDegrees = 14.61;
    /** Lateral offset of the Limelight from the center of the robot. */
    public static final double limelightLateralOffsetMeters = 0.152;
    /** Horizontal distance from limelight to front bumper. */
    public static final double limelightBumperDistanceMeters = 0.0;
  }
}