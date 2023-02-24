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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;


public class SwerveModule {
  public int moduleNumber;
  private double angleOffset;
  private double lastAngle;
  private CANCoder absoluteEncoder;
  private CANSparkMax mAngleMotor;
  private CANSparkMax mDriveMotor;
  private RelativeEncoder mDriveEncoder;

  //limits acceleration of drive motor in meters/seconds^2
  private SlewRateLimiter driveMotorSlew = new SlewRateLimiter(16.);

  public static PIDController mDrivePID =
    new PIDController(
      Constants.Swerve.drivekP, Constants.Swerve.drivekI, Constants.Swerve.drivekD);
  public static PIDController mAnglePID =
    new PIDController(
      Constants.Swerve.anglekP, Constants.Swerve.anglekI, Constants.Swerve.anglekD);

  SimpleMotorFeedforward mDriveFeedforward =
    new SimpleMotorFeedforward(
      Constants.Swerve.drivekS, Constants.Swerve.drivekV, Constants.Swerve.drivekA);
  

  public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int cancoderID, double angleOffset) {
    
    
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

    lastAngle = absoluteEncoder.getAbsolutePosition();
    SmartDashboard.putNumber("angleP", mAnglePID.getP());
    SmartDashboard.putNumber("lookAheadSeconds", 0.02);

    //mAnglePID.enableContinuousInput(-180, 180);
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
      double percentOutput =
        driveMotorSlew.calculate(desiredState.speedMetersPerSecond) / Constants.Swerve.maxVelocityMetersPerSecond;
      mDriveMotor.set(percentOutput);
    }
    


    
    //this is here so the wheel does not reset angle every time velocity is 0
    double targetWheelAngle =
      (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxVelocityMetersPerSecond * 0.01))
        ? lastAngle
        : desiredState.angle.getDegrees();

    double currentEncoderAngle = absoluteEncoder.getAbsolutePosition();

    //ensures no 360 degree rotations (i.e. getting from -179 to 179)
    if (targetWheelAngle - currentEncoderAngle > 180) {
      targetWheelAngle -= 360;
    }
    else if (targetWheelAngle - currentEncoderAngle < -180) {
      targetWheelAngle += 360;
    }

    mAngleMotor.setVoltage(mAnglePID.calculate(currentEncoderAngle, targetWheelAngle));

    lastAngle = targetWheelAngle;
  }

  public void resetAngleMotors() {
    double currentEncoderAngle = absoluteEncoder.getAbsolutePosition();

    mAngleMotor.setVoltage(mAnglePID.calculate(currentEncoderAngle, 0));
  }

  public SwerveModuleState getState() {
    //conversion factor is already set below to convert rpm of motor to m/s of wheel
    double velocity = mDriveEncoder.getVelocity();
    
    Rotation2d angle =
        Rotation2d.fromDegrees(
          absoluteEncoder.getAbsolutePosition());
            
    return new SwerveModuleState(velocity, angle);
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
      * Constants.Swerve.wheelCircumference
    );

    //converts rpm of motor to m/s of wheel
    mDriveEncoder.setVelocityConversionFactor(
      1./60. * Constants.Swerve.driveReduction
      * Constants.Swerve.wheelCircumference
    );

    mDriveMotor.burnFlash();
  }
}