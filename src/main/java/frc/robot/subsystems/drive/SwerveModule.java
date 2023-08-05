package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;


public class SwerveModule {

  private SwerveModuleIO io;
  public int moduleNumber;
  //this is generated by the @autolog annotation
  private SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  private static PIDController mDrivePID = new PIDController(
    Constants.Swerve.drivekPVoltsPerMeterPerSecond, 
    Constants.Swerve.drivekIVoltsPerMeter, 
    Constants.Swerve.drivekDVoltsPerMeterPerSecondSquared);
  private static PIDController mAnglePID = new PIDController(
    Constants.Swerve.anglekPVoltsPerDegree,
    Constants.Swerve.anglekIVoltsPerDegreeSeconds,
    Constants.Swerve.anglekDVoltsPerDegreePerSecond);

  private SimpleMotorFeedforward mDriveFeedforward = new SimpleMotorFeedforward(
    Constants.Swerve.drivekSVolts, 
    Constants.Swerve.drivekVVoltsSecondsPerMeter, 
    Constants.Swerve.drivekAVoltsSecondsSquaredPerMeter);

  public SwerveModule(SwerveModuleIO io, int moduleNumber) {

    this.io = io;
    this.moduleNumber = moduleNumber;

    mAnglePID.enableContinuousInput(-180, 180);
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  private SwerveModuleState constrainState(SwerveModuleState toConstrain) {
    double originalDegrees = toConstrain.angle.getDegrees();
    // Start by assuming there is no need to constrain.
    double constrainedDegrees = originalDegrees;
    if (originalDegrees < -180 || originalDegrees > 180) { //TODO: is this if statement necessary???
      // Constrain angle if necessary.
      constrainedDegrees = MathUtil.inputModulus(originalDegrees, -180, 180);
    }
    return new SwerveModuleState(toConstrain.speedMetersPerSecond, Rotation2d.fromDegrees(constrainedDegrees));
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean closedLoop) {
    // Make sure the desired state's angle is in the range [-180, 180], so it can be appropriately
    // compared to the current angle from the CANCoder.
    desiredState = constrainState(desiredState);
    desiredState = SwerveModuleState.optimize(
      desiredState,
      Rotation2d.fromDegrees(inputs.angleAbsolutePositionDegrees)
      );
    desiredState = constrainState(desiredState); // constrain one more time after optimization just to be safe, because I'm unsure if optimization can ever pull the angle out of [-180, 180]

    setDesiredStateNoOptimize(desiredState, closedLoop);
  }

  public void setDesiredStateNoOptimize(SwerveModuleState desiredState, boolean closedLoop) {
    if (closedLoop) {
      // Conversion factor is already set below to convert rpm of motor to m/s of wheel.
      double wheelMetersPerSecond = inputs.driveVelocityMetersPerSecond;

      double feedforward = mDriveFeedforward.calculate(desiredState.speedMetersPerSecond);
      double pidCorrection = mDrivePID.calculate(wheelMetersPerSecond, desiredState.speedMetersPerSecond);
      double outputVolts = MathUtil.clamp(feedforward + pidCorrection, -12, 12);

      io.setDriveVoltage(outputVolts);
    }
    else {
      io.setDriveVoltage(mDriveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }
    
    // If angle motors are messed up then check commit from 2/28 for changes.
    double targetWheelAngleDegrees = desiredState.angle.getDegrees();
    double currentEncoderAngleDegrees = inputs.angleAbsolutePositionDegrees;

    io.setAngleVoltage(mAnglePID.calculate(currentEncoderAngleDegrees, targetWheelAngleDegrees));
  }


  public SwerveModuleState getState() {
    // Conversion factor is already set below to convert rpm of motor to m/s of wheel.
    double velocityMetersPerSecond = inputs.driveVelocityMetersPerSecond;
    
    Rotation2d angle =
        Rotation2d.fromDegrees(
          inputs.angleAbsolutePositionDegrees); 
          // Absolute encode use this.
    return new SwerveModuleState(velocityMetersPerSecond, angle);
  }



  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
      inputs.drivePositionMeters, Rotation2d.fromDegrees(inputs.angleAbsolutePositionDegrees)
    );
    // Use absolute encoder for this.
  }

}