// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team1787_lib;

/**
 * A swerve module configuration.
 * <p>
 * We can store swerve module mechanical aspects here lol
 */
public class ModuleConfiguration {
  private final double wheelDiameter;
  private final double wheelCircumference;
  private final double driveReduction;
  private final boolean driveInverted;

  private final double steerReduction;
  private final boolean steerInverted;


  /**
   * Creates a new module configuration.
   *
   * @param wheelDiameter  The diameter of the module's wheel in meters.
   * @param driveReduction The overall drive reduction of the module. Multiplying motor rotations by this value
   *                       should result in wheel rotations.
   * @param driveInverted  Whether the drive motor should be inverted. If there is an odd number of gea reductions
   *                       this is typically true.
   * @param steerReduction The overall steer reduction of the module. Multiplying motor rotations by this value
   *                       should result in rotations of the steering pulley.
   * @param steerInverted  Whether the steer motor should be inverted. If there is an odd number of gear reductions
   *                       this is typically true.
   */
  public ModuleConfiguration(
    double wheelDiameter,
    double driveReduction,
    boolean driveInverted,
    double steerReduction,
    boolean steerInverted
  ) {
    this.wheelDiameter = wheelDiameter;
    this.driveReduction = driveReduction;
    this.driveInverted = driveInverted;
    this.steerReduction = steerReduction;
    this.steerInverted = steerInverted;
    this.wheelCircumference = Math.PI*wheelDiameter;
  }

  /**
   * Gets the diameter of the wheel in meters.
   */
  public double getWheelDiameter() {
    return wheelDiameter;
  }

  /**
   * Gets the circuference of the wheel in meters.
   */
  public double getWheelCircumference() {
    return wheelCircumference;
  }

  /**
   * Gets the overall reduction of the drive system.
   * <p>
   * If this value is multiplied by drive motor rotations the result would be drive wheel rotations.
   */
  public double getDriveReduction() {
      return driveReduction;
  }

  /**
   * Gets if the drive motor should be inverted.
   */
  public boolean isDriveInverted() {
      return driveInverted;
  }

  /**
   * Gets the overall reduction of the steer system.
   * <p>
   * If this value is multiplied by steering motor rotations the result would be steering pulley rotations.
   */
  public double getSteerReduction() {
      return steerReduction;
  }

  /**
   * Gets if the steering motor should be inverted.
   */
  public boolean isSteerInverted() {
      return steerInverted;
  }
  
}
