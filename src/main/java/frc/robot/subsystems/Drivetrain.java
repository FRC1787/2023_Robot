// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final AHRS gyro; // NavX connected over MXP
  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  private ChassisSpeeds desiredChassisSpeeds;

  public Drivetrain() {
    gyro = new AHRS(SPI.Port.kMXP);
    desiredChassisSpeeds = new ChassisSpeeds(0, 0, 0);

    mSwerveMods = new SwerveModule[] {
        new SwerveModule(
            0,
            Constants.Swerve.FrontLeftSwerveModule.driveMotorID,
            Constants.Swerve.FrontLeftSwerveModule.steerMotorID,
            Constants.Swerve.FrontLeftSwerveModule.steerEncoderID,
            Constants.Swerve.FrontLeftSwerveModule.steerOffset),
        new SwerveModule(
            1,
            Constants.Swerve.FrontRightSwerveModule.driveMotorID,
            Constants.Swerve.FrontRightSwerveModule.steerMotorID,
            Constants.Swerve.FrontRightSwerveModule.steerEncoderID,
            Constants.Swerve.FrontRightSwerveModule.steerOffset),
        new SwerveModule(
            2,
            Constants.Swerve.BackLeftSwerveModule.driveMotorID,
            Constants.Swerve.BackLeftSwerveModule.steerMotorID,
            Constants.Swerve.BackLeftSwerveModule.steerEncoderID,
            Constants.Swerve.BackLeftSwerveModule.steerOffset),
        new SwerveModule(
            3,
            Constants.Swerve.BackRightSwerveModule.driveMotorID,
            Constants.Swerve.BackRightSwerveModule.steerMotorID,
            Constants.Swerve.BackRightSwerveModule.steerEncoderID,
            Constants.Swerve.BackRightSwerveModule.steerOffset)
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRobotRotation2d(), getModulePositions());
    field = new Field2d();
  }

  // GYROSCOPE

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroYaw() {
    gyro.zeroYaw(); //this is the exact same thing as saying gyro.reset();
  }

  /**
   * Gets the angle of the robot measured by the gyroscope as a Rotation2d (continuous).
   * @return rotation2d - this angle will be counterclockwise positive.
   */
  public Rotation2d getRobotRotation2d() {
    return gyro.getRotation2d();
  }

  /**
   * Returns the current pitch value (in degrees, from -180 to 180) of the robot, based off of the NavX.
   * Pitch is a measure of angle between the robot-oriented X-axis and the horizontal.
   * @return The current pitch value in degrees (-180 to 180). This value will be positive if the front of the robot is raised.
   */
  public double getRobotPitchDegrees() {
    return 0.0; //TODO: unclear which axes of the gyro corresponds to the axes of the robot, do a test to verify after gyro omnimount
  }

  /**
   * Returns the current roll value (in degrees, from -180 to 180) of the robot, based off of the NavX.
   * Roll is a measure of angle between the robot-oriented Y-axis and the horizontal.
   * @return The current roll value in degrees (-180 to 180). This value will be positive if the left of the robot (positive Y) is raised.
   */
  public double getRobotRollDegrees() {
    return 0.0; //TODO: see above
  }

  public double getRobotPitchDegreesPerSecond() {
    return 0.0;
  }
  
  public double getRobotRollDegreesPerSecond() {
    return 0.0;
  }

  // POSE, FIELD, ODOMETRY

  /**
   * Gets the current position of the robot on the field in meters.
   * This value considers the origin to be the right side of the robot's current alliance.
   * <p>
   * A positive X value brings the robot towards the opposing alliance,
   * and a positive Y value brings the robot left as viewed by your alliance.
   * @return The current position of the robot on the field in meters.
   */
  public Pose2d getPoseMeters() {
    return swerveOdometry.getPoseMeters();
  }

  /**
   * Sets the current position of the robot on the field in meters.
   * <p>
   * A positive X value brings the robot towards the opposing alliance,
   * and a positive Y value brings the robot left as viewed by your alliance.
   * @param pose
   */
  public void setPoseMeters(Pose2d pose) {
    swerveOdometry.resetPosition(
      getRobotRotation2d(),
      getModulePositions(),
      pose
    );
  }

  public void updateOdometry() {
    swerveOdometry.update(getRobotRotation2d(), getModulePositions());
    field.setRobotPose(swerveOdometry.getPoseMeters());
  }

  // SWERVE MODULES

  /**
   * Drives the robot based on a desired ChassisSpeeds.
   * <p>
   * Takes in a robot relative ChassisSpeeds. Field relative control can be accomplished by using the ChassisSpeeds.fromFieldRelative() method.
   * @param desiredChassisSpeeds - Robot relative ChassisSpeeds object in meters per second and radians per second.
   */
  public void drive(ChassisSpeeds desiredChassisSpeeds, boolean closedLoop) {

    this.desiredChassisSpeeds = desiredChassisSpeeds;

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxAchievableVelocityMetersPerSecond);

    if (closedLoop)
      setModuleStatesClosedLoop(swerveModuleStates);

    else
      setModuleStatesOpenLoop(swerveModuleStates);
  }


  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStatesClosedLoop(SwerveModuleState[] desiredStates) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], true);
    }
  }

  //useful for debugging
  public void setModuleStatesOpenLoop(SwerveModuleState[] desiredStates) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  //required by odometry
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];

    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  @Override
  public void periodic() {
    updateOdometry();

    SmartDashboard.putNumber("pose x meters", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("pose y meters", swerveOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("pose rotation degrees", swerveOdometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("gyro yaw degrees", getRobotRotation2d().getDegrees());
    SmartDashboard.putNumber("pitch degrees", getRobotPitchDegrees());
    SmartDashboard.putNumber("roll degrees", getRobotRollDegrees());
    SmartDashboard.putNumber("pitch degrees per second", getRobotPitchDegreesPerSecond());
    SmartDashboard.putNumber("roll degrees per second", getRobotRollDegreesPerSecond());
    SmartDashboard.putData("field", field);

    SmartDashboard.putNumber("DesiredChassisSpeedsXMetersPerSecond", desiredChassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("DesiredChassisSpeedsXMetersPerSecond", desiredChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("DesiredChassisSpeedsRotationRadiansPerSecond", desiredChassisSpeeds.omegaRadiansPerSecond);
  }
}