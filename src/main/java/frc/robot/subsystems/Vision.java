// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */

  NetworkTable limelight;

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
    changePipeline(target);

    return (target.heightMeters - Constants.Vision.limelightHeightMeters)
        / Math.tan(
            Math.toRadians(getTYDegrees() + Constants.Vision.limelightAngleDegrees));
  }

  public double getLateralOffsetMeters(Constants.Vision.LimelightTarget target) {
    changePipeline(target);

    return getTargetDistanceMeters(target) * Math.tan(Math.toRadians(getTXDegrees()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("apriltag lateral offset", getLateralOffsetMeters(Constants.Vision.LimelightTarget.midTape));
    SmartDashboard.putNumber("tarfget distance meters", getTargetDistanceMeters(Constants.Vision.LimelightTarget.midTape));
  }
}
