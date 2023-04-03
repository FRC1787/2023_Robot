// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {

  private final AHRS gyro; // NavX connected over MXP
  private CustomSwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  private ChassisSpeeds desiredChassisSpeeds;

  private SlewRateLimiter chassisSpeedsXSlewLimiter;
  private SlewRateLimiter chassisSpeedsYSlewLimiter;

  public Drivetrain() { 
    gyro = new AHRS(Port.kUSB); //new AHRS(Port.kUSB, AHRS.SerialDataType.kRawData, (byte)200); Using the USB Port prevents reading of "raw" data, including pitch and roll velocity :(
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

    swerveOdometry = new CustomSwerveDriveOdometry(Constants.Swerve.swerveKinematics, getRobotRotation2d(), getModulePositions());
    gyro.zeroYaw();
    swerveOdometry.resetPosition(getRobotRotation2d(), getModulePositions(), new Pose2d(0, 0, new Rotation2d(0)));
    System.out.println("Pose init to 0 confirmed!");
    field = new Field2d();

    chassisSpeedsXSlewLimiter = new SlewRateLimiter(Constants.Swerve.maxDesiredDriverAccel);
    chassisSpeedsYSlewLimiter = new SlewRateLimiter(Constants.Swerve.maxDesiredDriverAccel);
  }

  // GYROSCOPE

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroYaw() {
    gyro.zeroYaw(); //this is the exact same thing as saying gyro.reset();
    gyro.setAngleAdjustment(0);

    swerveOdometry.resetPosition(getRobotRotation2d(), getModulePositions(), getPoseMeters());
  }

  /**
   * Sets the gyroscope angle adjustment to be 180.
   * In other words, if the robot is facing towards the grid, calling this method will field orient it.
   */
  public void setGyroscope180() {
    gyro.zeroYaw();
    gyro.setAngleAdjustment(180);

    swerveOdometry.resetPosition(getRobotRotation2d(), getModulePositions(), getPoseMeters());
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
    return gyro.getRoll(); //since navx is mounted silly and also axis convention this is correct
    //also the navx comments are bad don't trust them
  }

  /**
   * Returns the current roll value (in degrees, from -180 to 180) of the robot, based off of the NavX.
   * Roll is a measure of angle between the robot-oriented Y-axis and the horizontal.
   * @return The current roll value in degrees (-180 to 180). This value will be positive if the left of the robot (positive Y) is raised.
   */
  public double getRobotRollDegrees() {
    return gyro.getPitch(); //since navx is mounted silly and also axis convention this is correct
    //also the navx comments are bad don't trust them
  }

  /**
   * Returns the current angular pitch velocity in degrees per second of the robot, based off of hte NavX.
   * This represents how quickly the angle between the robot-oriented X-axis and the horizontal changes.
   * @return The current pitch speed value in degrees per second. If the front of the robot is being raised, this will return a positive value.
   */
  public double getRobotPitchDegreesPerSecond() {
    return gyro.getRawGyroY();
  }
  
  /**
   * Returns the current angular roll velocity in degrees per second of the robot, based off of the NavX.
   * This represents how quickly the angle between the robot-oriented Y-axis and the horizontal changes.
   * @return The current roll speed value in degrees per second. If the left side of the robot is being raised, this will return a positive value.
   */
  public double getRobotRollDegreesPerSecond() {
    return gyro.getRawGyroX();
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

    desiredChassisSpeeds.vxMetersPerSecond = chassisSpeedsXSlewLimiter.calculate(desiredChassisSpeeds.vxMetersPerSecond);
    desiredChassisSpeeds.vyMetersPerSecond = chassisSpeedsYSlewLimiter.calculate(desiredChassisSpeeds.vyMetersPerSecond);

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

  public void setModuleStatesClosedLoopNoOptimize(SwerveModuleState[] desiredStates) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredStateNoOptimize(desiredStates[mod.moduleNumber], true);
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
    SmartDashboard.putNumber("DesiredChassisSpeedsYMetersPerSecond", desiredChassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("DesiredChassisSpeedsRotationRadiansPerSecond", desiredChassisSpeeds.omegaRadiansPerSecond);
    
    // SmartDashboard.putNumber("front left module speed", mSwerveMods[0].getState().speedMetersPerSecond);
  
    SmartDashboard.putNumber("front left distance meters", mSwerveMods[0].getPosition().distanceMeters);
    SmartDashboard.putNumber("front right distance meters", mSwerveMods[1].getPosition().distanceMeters);
    SmartDashboard.putNumber("back left distance meters", mSwerveMods[2].getPosition().distanceMeters);
    SmartDashboard.putNumber("back right distance meters", mSwerveMods[3].getPosition().distanceMeters);
  
    SmartDashboard.putNumber("front left degrees", mSwerveMods[0].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("front right degrees", mSwerveMods[1].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("back left degrees", mSwerveMods[2].getPosition().angle.getDegrees());
    SmartDashboard.putNumber("back right degrees", mSwerveMods[3].getPosition().angle.getDegrees());

    // SmartDashboard.putNumber("front right cancoder")
    
    SmartDashboard.putNumber("front left velocity", mSwerveMods[0].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("front right velocity", mSwerveMods[1].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("back left velocity", mSwerveMods[2].getState().speedMetersPerSecond);
    SmartDashboard.putNumber("back right velocity", mSwerveMods[3].getState().speedMetersPerSecond);


    SmartDashboard.putBoolean("gyro is calibrating", gyro.isCalibrating());
  }
}