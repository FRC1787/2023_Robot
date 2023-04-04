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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;


public class SwerveModule {
  public int moduleNumber;
  private double angleOffset;
  private CANCoder absoluteEncoder;
  private CANSparkMax mAngleMotor;
  private CANSparkMax mDriveMotor;
  private RelativeEncoder mDriveEncoder;
  private RelativeEncoder mAngleEncoder;

  public static PIDController mDrivePID;
  public static PIDController mAnglePID;
  SimpleMotorFeedforward mDriveFeedforward;

  public SwerveModule(int moduleNumber, int driveMotorID, int angleMotorID, int cancoderID, double angleOffset) {
    mDrivePID = new PIDController(
      Constants.Swerve.drivekPVoltsPerMeterPerSecond, 
      Constants.Swerve.drivekIVoltsPerMeter, 
      Constants.Swerve.drivekDVoltsPerMeterPerSecondSquared);

    mAnglePID = new PIDController(
      Constants.Swerve.anglekPVoltsPerDegree, Constants.Swerve.anglekIVoltsPerDegreeSeconds, Constants.Swerve.anglekDVoltsPerDegreePerSecond);
    
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
    mAngleEncoder = mAngleMotor.getEncoder();
    configAngleMotor();

    //this encoder will always have units of degrees of the wheel
    mAngleEncoder.setPosition(absoluteEncoder.getAbsolutePosition());

    /* Drive Motor Config */
    mDriveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    mDriveEncoder = mDriveMotor.getEncoder();
    configDriveMotor();
    
    mAnglePID.enableContinuousInput(-180, 180);
  }

  private SwerveModuleState constrainState(SwerveModuleState toConstrain) {
    double originalDegrees = toConstrain.angle.getDegrees();
    double constrainedDegrees = originalDegrees; // start by assuming there is no need to constrain
    if (originalDegrees < -180 || originalDegrees > 180) {
      // constrain angle if necessary
      constrainedDegrees = MathUtil.inputModulus(originalDegrees, -180, 180);
    }
    return new SwerveModuleState(toConstrain.speedMetersPerSecond, Rotation2d.fromDegrees(constrainedDegrees));
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean closedLoop) {
    // make sure the desired state's angle is in the range [-180, 180], so it can be appropriately
    // compared to the current angle from the CANCoder:
    desiredState = constrainState(desiredState);
    desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(absoluteEncoder.getAbsolutePosition()));
    desiredState = constrainState(desiredState); // constrain one more time after optimization just to be safe, because I'm unsure if optimization can ever pull the angle out of [-180, 180]

    setDesiredStateNoOptimize(desiredState, closedLoop);
  }

  public void setDesiredStateNoOptimize(SwerveModuleState desiredState, boolean closedLoop) {
    if (closedLoop) {
      //conversion factor is already set below to convert rpm of motor to m/s of wheel
      double wheelMetersPerSecond = mDriveEncoder.getVelocity();

      double feedforward = mDriveFeedforward.calculate(desiredState.speedMetersPerSecond);
      double pidCorrection = mDrivePID.calculate(wheelMetersPerSecond, desiredState.speedMetersPerSecond);
      double outputVolts = MathUtil.clamp(feedforward + pidCorrection, -12, 12);

      mDriveMotor.setVoltage(outputVolts);
    }
    else {
      mDriveMotor.setVoltage(mDriveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }
    
    //if angle motors are messed up then check commit from 2/28 for changes
    double targetWheelAngleDegrees = desiredState.angle.getDegrees();
    double currentEncoderAngleDegrees = absoluteEncoder.getAbsolutePosition();

    mAngleMotor.setVoltage(mAnglePID.calculate(currentEncoderAngleDegrees, targetWheelAngleDegrees));
  }


  public SwerveModuleState getState() {
    //conversion factor is already set below to convert rpm of motor to m/s of wheel
    double velocityMetersPerSecond = mDriveEncoder.getVelocity();
    
    Rotation2d angle =
        Rotation2d.fromDegrees(
          mAngleEncoder.getPosition()); // TODO: Maybe try CANCoder here? or would that mess with callers expecting continuous angle measurements?
            
    return new SwerveModuleState(velocityMetersPerSecond, angle);
  }



  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      mDriveEncoder.getPosition(), Rotation2d.fromDegrees(mAngleEncoder.getPosition())
      // TODO: Maybe try CANCoder here? or would that mess with callers expecting continuous angle measurements?
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
    mAngleEncoder.setPositionConversionFactor(Constants.Swerve.steerReduction*360.0);
    mAngleEncoder.setVelocityConversionFactor(Constants.Swerve.steerReduction*360.0/60.0);
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

  public void zeroDriveEncoder() {
    mDriveEncoder.setPosition(0);
  }
}