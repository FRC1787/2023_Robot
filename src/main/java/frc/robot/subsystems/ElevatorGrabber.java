// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorGrabber extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax elevatorMotor;
  private CANSparkMax grabberMotor;

  private RelativeEncoder encoder;
  private DoubleSolenoid solenoid;
  
  private DigitalInput lowerLimitSwitch;

  private PIDController velocityController;
  private SimpleMotorFeedforward feedforward;

  public ElevatorGrabber() {
    elevatorMotor = new CANSparkMax(
        Constants.ElevatorGrabber.elevatorMotorID,
        MotorType.kBrushless);

    grabberMotor = new CANSparkMax(
        Constants.ElevatorGrabber.grabberMotorID,
        MotorType.kBrushless);

    encoder = elevatorMotor.getAlternateEncoder(4096);
    encoder.setPositionConversionFactor(Constants.ElevatorGrabber.grabberMetersPerRotation);
    encoder.setVelocityConversionFactor(Constants.ElevatorGrabber.grabberMetersPerRotation);

    solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        Constants.ElevatorGrabber.elevatorRetractPneumaticChannel,
        Constants.ElevatorGrabber.elevatorExtendPneumaticChannel);

    lowerLimitSwitch = new DigitalInput(Constants.ElevatorGrabber.lowerLimitSwitchID);

    velocityController = new PIDController(
        Constants.ElevatorGrabber.kPVoltsPerMeterPerSecond,
        Constants.ElevatorGrabber.kIVoltsPerMeter,
        Constants.ElevatorGrabber.kDVoltsPerMeterPerSecondSquared);

    feedforward = new SimpleMotorFeedforward(
      Constants.ElevatorGrabber.kSVolts,
      Constants.ElevatorGrabber.kVVoltSecondsPerMeter);

    elevatorMotor.setSmartCurrentLimit(60);
    setMotorCurrentLimits();
    setMotorInversions();
  }

  private void setMotorInversions() {
    //TODO: set motor inversions
  }

  private void setMotorCurrentLimits() {
    //TODO: fix amp limit
    elevatorMotor.setSmartCurrentLimit(0);
  }

  private boolean atLowerLimit() {
    return lowerLimitSwitch.get();
  }

  private boolean atUpperLimit() {
    //returns true if elevator position is greater than max elevator height (1.72 meters)
    return getElevatorPositionMeters() >= 1.7272;
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
  public void setElevatorMotorMetersPerSecond(double targetMetersPerSecond) {
    double currentMetersPerSecond = getElevatorVelocityMetersPerSecond();

    double pidOutput = velocityController.calculate(currentMetersPerSecond, targetMetersPerSecond);
    double feedforwardOutput = feedforward.calculate(targetMetersPerSecond);

    double totalOutput = pidOutput + feedforwardOutput;

    if (atLowerLimit() && totalOutput < 0)
      totalOutput = 0;

    if (atUpperLimit() && totalOutput > 0)
      totalOutput = 0;

    setElevatorMotorVolts(totalOutput);
  }

  public void extendPneumatics() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPneumatics() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Sets voltage of the grab motor.
   * @param voltage - make this positive to intake a cone/outtake a cube,
   * while negative to outtake a cone/intake a cube 
   */
  public void setGrabMotorVolts(double voltage) {
    grabberMotor.setVoltage(voltage);
  }

  public double getElevatorPositionMeters() {
    return encoder.getPosition();
  }

  public void zeroEncoder() {
    encoder.setPosition(0);
  }

  public double getGrabOutputAmps() {
    return grabberMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    if (atLowerLimit()) {
      zeroEncoder();
    }
  }
}