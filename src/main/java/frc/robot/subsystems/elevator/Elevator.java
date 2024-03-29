// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import frc.robot.Constants;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Elevator extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax elevatorMotor;

  private RelativeEncoder encoder;
  
  private DigitalInput lowerLimitSwitch;

  // This PID controller is used to follow the trapezoidal motion profile generated by the MoveElevatorToPostion command.
  private PIDController velocityController;

  private ElevatorFeedforward feedforward;

  public double desiredVelocity;

  private boolean hasBeenHomed;

  public Elevator() {
    elevatorMotor = new CANSparkMax(
        Constants.ElevatorGrabber.elevatorMotorID,
        MotorType.kBrushless);

    // encoder = elevatorMotor.getAlternateEncoder(8192);
    // encoder.setPositionConversionFactor(1);
    // encoder.setVelocityConversionFactor(1);

    encoder = elevatorMotor.getEncoder();
    configureMotors();
  

    // SysId recommends an depth of 5 - 10 samples per average (we picked 8 because 2^3).
    // encoder.setAverageDepth(8);
    // encoder.setMeasurementPeriod(0);


    lowerLimitSwitch = new DigitalInput(Constants.ElevatorGrabber.lowerLimitSwitchID);

    velocityController = new PIDController(
        Constants.ElevatorGrabber.kPVoltsPerMeterPerSecond,
        Constants.ElevatorGrabber.kIVoltsPerMeter,
        Constants.ElevatorGrabber.kDVoltsPerMeterPerSecondSquared);

    feedforward = new ElevatorFeedforward(
      Constants.ElevatorGrabber.kSVolts,
      Constants.ElevatorGrabber.kGVolts,
      Constants.ElevatorGrabber.kVVoltsPer_MeterPerSecond,
      Constants.ElevatorGrabber.kAVoltsPer_MeterPerSecondSquared);

    desiredVelocity = 0;

    hasBeenHomed = false;

  }

  private void configureMotors() {
    elevatorMotor.restoreFactoryDefaults();
    elevatorMotor.setSmartCurrentLimit(50);
    elevatorMotor.setInverted(true);
    elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    elevatorMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    encoder.setPositionConversionFactor(Constants.ElevatorGrabber.grabberMetersPerRotation * Constants.ElevatorGrabber.elevatorReduction);
    encoder.setVelocityConversionFactor(Constants.ElevatorGrabber.grabberMetersPerSecondPerRPM * Constants.ElevatorGrabber.elevatorReduction);
    elevatorMotor.burnFlash();
  }


  private boolean atLowerLimit() {
    return lowerLimitSwitch.get();
  }

  private boolean atUpperLimit() {
    // Returns true if elevator position is greater than max elevator height (1.72 meters).
    return getElevatorPositionMeters() >= 1.71;
  }

  public void setElevatorMotorVolts(double volts) {
    elevatorMotor.setVoltage(volts);
  }

  public double getElevatorVelocityMetersPerSecond() {
    return encoder.getVelocity();
  }

  /**
   * Sets velocity of the elevator
   * @param targetMetersPerSecond - Make this positive if you want to extend the elevator up.
   */
  public void setElevatorMotorMetersPerSecond(double targetMetersPerSecond, double targetMetersPerSecondSquared) {
    desiredVelocity = targetMetersPerSecond;
    
    double currentMetersPerSecond = getElevatorVelocityMetersPerSecond();

    double pidOutput = velocityController.calculate(currentMetersPerSecond, targetMetersPerSecond);
    double feedforwardOutput = feedforward.calculate(targetMetersPerSecond, targetMetersPerSecondSquared);

    double totalOutput = pidOutput + feedforwardOutput;

    // TODO: this could be just one if statement with an OR operator.
    // Stops elevator at limits.
    if (atLowerLimit() && totalOutput < 0)
      totalOutput = 0;

    if (atUpperLimit() && totalOutput > 0)
      totalOutput = 0;

    setElevatorMotorVolts(totalOutput);
  }

  public double getElevatorPositionMeters() {
    return encoder.getPosition();
  }

  public void zeroEncoder() {
    encoder.setPosition(0);
  }

  public boolean hasBeenHomed() {
    return hasBeenHomed;
  }

  @Override
  public void periodic() {
    if (atLowerLimit()) {
      zeroEncoder();
      hasBeenHomed = true;
    }

    SmartDashboard.putNumber("elevator position meters", getElevatorPositionMeters());
    SmartDashboard.putNumber("motor encoder position", elevatorMotor.getEncoder().getPosition());
    SmartDashboard.putBoolean("elevator limit switch", atLowerLimit());
    SmartDashboard.putNumber("elevator speed meters per second", getElevatorVelocityMetersPerSecond());

    SmartDashboard.putNumber("relative encoder position", encoder.getPosition());
    SmartDashboard.putNumber("relative encoder velocity", encoder.getVelocity());
  }
}
