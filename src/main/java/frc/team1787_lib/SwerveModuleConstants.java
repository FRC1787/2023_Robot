// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1787_lib;

/** Add your docs here. */
public class SwerveModuleConstants {
  /** CAN ID for the motor controller used to control driving motor */
  public final int driveMotorID;
  /** CAN ID for the motor controller used to control steering motor */
  public final int steerMotorID;
  /** CAN ID for absolute encoder */
  public final int steerEncoderID;
  /** Offset from true zero for the front left swerve module in radians */
  public final double steerOffset;
  

  /**
   * Swerve Module constants used to create a swerve module
   * @param driverMotorID CAN ID for the motor controller used to control driving motor
   * @param steerMotorID CAN ID for the motor controller used to control steering motor
   * @param steerEncoderID CAN ID for absolute encoder
   * @param steerOffset Offset from true zero for the front left swerve module in radians
   */
  public SwerveModuleConstants(int driverMotorID, int steerMotorID, int steerEncoderID, double steerOffset) {
    this.driveMotorID = driverMotorID;
    this.steerMotorID = steerMotorID;
    this.steerEncoderID = steerEncoderID;
    this.steerOffset = steerOffset;
  }
}
