// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
  private NetworkTable limelight;

  public Vision() {
    // Get the Limelight table from the NetworkTables instance.
    this.limelight = NetworkTableInstance.getDefault().getTable("limelight");
  }


  /**
   * Sets the Limelight pipeline to the specified target.
   * 
   * @param target The target pipeline to set the Limelight pipeline to.
   */
  public void setPipeline(Constants.LimelightConstants.LimelightTarget target) {
    limelight.getEntry("pipeline").setNumber(target.limelightPipeline);
  }

  /**
   * Gets the current pipeline the Llimelight is set to.
   * @return The current pipeline the Limelight is set to.
   */
  public int getPipeline() {
    return limelight.getEntry("pipeline").getNumber(0).intValue();
  }

  /**
   * Gets the horizontal offset from the Limelight's crosshair to the current target.
   * @return The horizontal offset from the Limelight's crosshair to the current target.
   */
  public double getTXDegrees() {
    return limelight.getEntry("tx").getDouble(0.0);
  }

  /**
   * Gets the vertical offset from the Limelight's crosshair to the current target.
   * @return The vertical offset from the Limelight's crosshair to the current target.
   */
  public double getTYDegrees() {
    return limelight.getEntry("ty").getDouble(0.0);
  }

  /**
   * Gets the distance in meters that the robot is from the currently targeted object.
   * @param target The target to get the distance from.
   * @return The distance in meters that the robot is from the currently targeted object.
   */
  public double getTargetDistanceMeters(Constants.LimelightConstants.LimelightTarget target) {
    setPipeline(target);


    return (target.heightMeters - Constants.LimelightConstants.limelightHeightMeters)
      / Math.tan(
        Math.toRadians(getTYDegrees() + Constants.LimelightConstants.limelightAngleDegrees)
      );    
  }

  /**
   * Gets the lateral offset in meters from the robot to the currently targeted object.
   * @param target The target to get the lateral offset from.
   * @return The lateral offset in meters from the robot to the currently targeted object.
   */
  public double getLateralOffsetMeters(Constants.LimelightConstants.LimelightTarget target) {
    setPipeline(target);
    return getTargetDistanceMeters(target) * Math.tan(Math.toRadians(getTXDegrees()));
  }

  @Override
  public void periodic() {}
}
