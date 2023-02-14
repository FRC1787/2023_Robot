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

  private final static AHRS gyro = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP
  public SwerveDriveOdometry swerveOdometry;
  public SwerveModule[] mSwerveMods;

  public Field2d field;

  // These are your swerve modules. They will probably cause hair loss?
  private final SwerveModule m_frontLeftModule;
  private final SwerveModule m_frontRightModule;
  private final SwerveModule m_backLeftModule;
  private final SwerveModule m_backRightModule;

  public Drivetrain() {
    field = new Field2d();

    mSwerveMods = new SwerveModule[] {
        m_frontLeftModule = new SwerveModule(
            0,
            Constants.Swerve.FrontLeftSwerveModule.driveMotorID,
            Constants.Swerve.FrontLeftSwerveModule.steerMotorID,
            Constants.Swerve.FrontLeftSwerveModule.steerEncoderID,
            Constants.Swerve.FrontLeftSwerveModule.steerOffset),
        m_frontRightModule = new SwerveModule(
            1,
            Constants.Swerve.FrontRightSwerveModule.driveMotorID,
            Constants.Swerve.FrontRightSwerveModule.steerMotorID,
            Constants.Swerve.FrontRightSwerveModule.steerEncoderID,
            Constants.Swerve.FrontRightSwerveModule.steerOffset),
        m_backLeftModule = new SwerveModule(
            2,
            Constants.Swerve.BackLeftSwerveModule.driveMotorID,
            Constants.Swerve.BackLeftSwerveModule.steerMotorID,
            Constants.Swerve.BackLeftSwerveModule.steerEncoderID,
            Constants.Swerve.BackLeftSwerveModule.steerOffset),
        m_backRightModule = new SwerveModule(
            3,
            Constants.Swerve.BackRightSwerveModule.driveMotorID,
            Constants.Swerve.BackRightSwerveModule.steerMotorID,
            Constants.Swerve.BackRightSwerveModule.steerEncoderID,
            Constants.Swerve.BackRightSwerveModule.steerOffset)
    };

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroscopeRotation(), getPositions());
  }

  // GYROSCOPE

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public static void zeroGyroscope() {
    gyro.zeroYaw();
  }

  /**
   * Gets the angle measured by the gyroscope (continuous).
   */
  public static Rotation2d getGyroscopeRotation() {
    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    // return Rotation2d.fromDegrees(-gyro.getAngle());
    return gyro.getRotation2d();
  }

  public double getPitch() {
    return gyro.getPitch();
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  public double getAngularSpeed() {
    return gyro.getRawGyroZ();
  }

  // POSE, FIELD, ODOMETRY

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void setOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(
        getGyroscopeRotation(),
        getPositions(),
        pose);
  }

  public void updateOdometry() {
    swerveOdometry.update(getGyroscopeRotation(), getPositions());
    field.setRobotPose(swerveOdometry.getPoseMeters());
  }

  // SWERVE MODULES

  public void drive(ChassisSpeeds chassisSpeeds) {

    SmartDashboard.putNumber("ChassisSpeedsX", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeedsY", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeedsRotation", chassisSpeeds.omegaRadiansPerSecond);

    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxVelocityMetersPerSecond);

    setModuleStatesClosedLoop(swerveModuleStates);
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStatesClosedLoop(SwerveModuleState[] desiredStates) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], true);
    }
  }

  public void setModuleStatesOpenLoop(SwerveModuleState[] desiredStates) {
    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void setAngleToZero() {
    for (SwerveModule mod : mSwerveMods) {
      mod.resetAngleMotors();
    }
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];

    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (SwerveModule mod : mSwerveMods) {
      positions[mod.moduleNumber] = mod.getPosition();
    }
    return positions;
  }

  // CONTROL

  /**
   * Returns 0 if the parameter {@code num} is lower than {@code deadzone} to
   * prevent joystick drift
   * 
   * @param num      Axis input value
   * @param deadzone Lowest value before input is set to 0
   * @return Axis input checked against deadzone value
   */
  public static double deadzone(double num, double deadzone) {
    return Math.abs(num) > deadzone ? num : 0;
  }

  /**
   * Adds a deadzone to axis input and squares the input
   * 
   * @param value Axis input
   * @return Squared and deadzoned input
   */
  public static double modifyAxis(double value) {
    value = deadzone(value, Constants.Controller.controllerDeadzone);

    // Square the axis
    // value = Math.copySign(Math.pow(value, 4), value);
    // value = (value * .50) + Math.pow(value * .50 , 3);
    return Math.signum(value) * Math.pow(value, 2);
  }

  // DRIVE COMMANDS
  public void rotateAtSpeed(double speedDegreesPerSecond) {
    drive(
        new ChassisSpeeds(
            0, 0, Math.toRadians(speedDegreesPerSecond)));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("pose x", swerveOdometry.getPoseMeters().getX());
    SmartDashboard.putNumber("pose y", swerveOdometry.getPoseMeters().getY());
    SmartDashboard.putNumber("pose rotation", swerveOdometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("pitch", gyro.getPitch());

    if (this.getCurrentCommand() != null) {
      SmartDashboard.putString("current drivetrain command", this.getCurrentCommand().getName());
    }

    SmartDashboard.putNumber("wheel velocity", getStates()[0].speedMetersPerSecond);

    updateOdometry();

    SmartDashboard.putNumber("encoder left", m_frontLeftModule.getState().angle.getDegrees());

    SmartDashboard.putNumber("front left swerve module speed", m_frontLeftModule.getState().speedMetersPerSecond);
    SmartDashboard.putNumber("gyro", getGyroscopeRotation().getDegrees());
  }
}