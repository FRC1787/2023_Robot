package frc.robot.subsystems;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;


public class SwerveModule {
  public int moduleNumber;
  private double angleOffset;
  private double lastAngleDegrees;
  private CANCoder absoluteEncoder;
  private CANSparkMax mAngleMotor;
  private CANSparkMax mDriveMotor;
  private RelativeEncoder mDriveEncoder;

  //limits acceleration of drive motor in meters/seconds^2
  private SlewRateLimiter driveMotorSlew;

  public static PIDController mDrivePID;
  public static PIDController mAnglePID;
  SimpleMotorFeedforward mDriveFeedforward;

  public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int cancoderID, double angleOffset) {
    driveMotorSlew = new SlewRateLimiter(16.);

    mDrivePID = new PIDController(
      Constants.Swerve.drivekPVoltsPerMeterPerSecond, 
      Constants.Swerve.drivekIVoltsPerMeterPerSecondSquared, 
      Constants.Swerve.drivekDVoltsPerMeter);

    mAnglePID = new PIDController(
      Constants.Swerve.anglekPVoltsPerDegree, Constants.Swerve.anglekIVolts, Constants.Swerve.anglekDVoltsPerDegreePerSecond);
    
    mDriveFeedforward = new SimpleMotorFeedforward(
      Constants.Swerve.drivekSVolts, 
      Constants.Swerve.drivekVVoltsSecondsPerMeter, 
      Constants.Swerve.drivekAVoltsSecondsSquaredPerMeter);

    this.moduleNumber = moduleNumber;
    this.angleOffset = angleOffset;

    /* Angle Encoder Config */
    absoluteEncoder = new CANCoder(cancoderID);
    configCANCoder();

    /* Angle Motor Config */
    mAngleMotor = new CANSparkMax(angleMotorID, MotorType.kBrushless);
    configAngleMotor();

    /* Drive Motor Config */
    mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    mDriveEncoder = mDriveMotor.getEncoder();
    configDriveMotor();

    lastAngleDegrees = absoluteEncoder.getAbsolutePosition();
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean closedLoop) {
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()));

    if (closedLoop) {
      //conversion factor is already set below to convert rpm of motor to m/s of wheel
      double wheelMetersPerSecond = mDriveEncoder.getVelocity();
      double desiredStateSpeedMetersPerSecond = driveMotorSlew.calculate(desiredState.speedMetersPerSecond);

      double feedforward = mDriveFeedforward.calculate(desiredStateSpeedMetersPerSecond);
      double pidCorrection = MathUtil.clamp(
        mDrivePID.calculate(wheelMetersPerSecond, desiredStateSpeedMetersPerSecond),
        -6, 6);

      mDriveMotor.setVoltage(feedforward + pidCorrection);
    }
    else {
      mDriveMotor.setVoltage(mDriveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }
    
    //this is here so the wheel does not reset angle every time velocity is 0
    double targetWheelAngleDegrees =
      (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxAchievableVelocityMetersPerSecond * 0.01))
        ? lastAngleDegrees
        : desiredState.angle.getDegrees();

    double currentEncoderAngleDegrees = absoluteEncoder.getAbsolutePosition();

    //ensures no 360 degree rotations (i.e. getting from -179 to 179)
    if (targetWheelAngleDegrees - currentEncoderAngleDegrees > 180) {
      targetWheelAngleDegrees -= 360;
    }
    else if (targetWheelAngleDegrees - currentEncoderAngleDegrees < -180) {
      targetWheelAngleDegrees += 360;
    }

    mAngleMotor.setVoltage(mAnglePID.calculate(currentEncoderAngleDegrees, targetWheelAngleDegrees));

    lastAngleDegrees = targetWheelAngleDegrees;
  }

  public SwerveModuleState getState() {
    //conversion factor is already set below to convert rpm of motor to m/s of wheel
    double velocityMetersPerSecond = mDriveEncoder.getVelocity();
    
    Rotation2d angle =
        Rotation2d.fromDegrees(
          absoluteEncoder.getAbsolutePosition());
            
    return new SwerveModuleState(velocityMetersPerSecond, angle);
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      mDriveEncoder.getPosition(), Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition())
    );
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition());
  }

  private void configCANCoder() {
    absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
    absoluteEncoder.configMagnetOffset(angleOffset);
    absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
  }

  private void configAngleMotor() {
    mAngleMotor.restoreFactoryDefaults();
    mAngleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    mAngleMotor.setInverted(Constants.Swerve.angleInvert);
    mAngleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    mAngleMotor.burnFlash();
  }

  private void configDriveMotor() {
    mDriveMotor.restoreFactoryDefaults();
    mDriveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    mDriveMotor.setInverted(Constants.Swerve.driveInvert);
    mDriveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    
    mDriveEncoder.setPosition(0.0);

    //converts rotations of motor to meters traveled of wheel
    mDriveEncoder.setPositionConversionFactor(
      Constants.Swerve.driveReduction
      * Constants.Swerve.wheelCircumferenceMeters
    );

    //converts rpm of motor to m/s of wheel
    mDriveEncoder.setVelocityConversionFactor(
      1./60. * Constants.Swerve.driveReduction
      * Constants.Swerve.wheelCircumferenceMeters
    );

    mDriveMotor.burnFlash();
  }
}