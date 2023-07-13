// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  private static NetworkTable limelight;

  public Vision() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }

  public void changePipeline(Constants.Vision.LimelightTarget target) {
    limelight.getEntry("pipeline").setNumber(target.limelightPipeline);
  }

  public double getTXDegrees() {
    return limelight.getEntry("tx").getDouble(0.0);
  }

  public double getTYDegrees() {
    return limelight.getEntry("ty").getDouble(0.0);
  }

  public double getTargetDistanceMeters(Constants.Vision.LimelightTarget target) {

    return (target.heightMeters - Constants.Vision.limelightHeightMeters)
        / Math.tan(
            Math.toRadians(getTYDegrees() + Constants.Vision.limelightAngleDegrees));
  }

  public double getLateralOffsetMeters(Constants.Vision.LimelightTarget target) {
    return getTargetDistanceMeters(target) * Math.tan(Math.toRadians(getTXDegrees()));
  }



  //the next three methods are static because drivetrain references them
  //gets pose using limelight 3d tingies
  public static Pose2d getLimelightPose2d() {

    double[] defaultValues = {0, 0, 0, 0, 0, 0};
    //limelight sends its pose estimate as an array of {x, y, z, roll, pitch, yaw} (in meters and degrees)
    double[] limelightPoseArray = limelight.getEntry("botpose_wpiblue").getDoubleArray(defaultValues);

    return new Pose2d(
      limelightPoseArray[0],
      limelightPoseArray[1],
      Rotation2d.fromDegrees(limelightPoseArray[5]));
  }

  public static double getTotalLatencyMs() {
    //targeting latency (ms)
    double tl = limelight.getEntry("tl").getDouble(0.0);
    //capture latency (ms)
    double cl = limelight.getEntry("cl").getDouble(0.0);

    return tl + cl;
  }

  //idk if this is the best way of implementing this but whatever
  public static boolean limelightSeesAprilTag() {
    //ta is the area on the camera of the target it sees
    double ta = limelight.getEntry("ta").getDouble(0.0);
    if (ta != 0) return true;
    else return false;
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("apriltag lateral offset", getLateralOffsetMeters(Constants.Vision.LimelightTarget.midTape));
    // SmartDashboard.putNumber("tarfget distance meters", getTargetDistanceMeters(Constants.Vision.LimelightTarget.midTape));
  }
}
