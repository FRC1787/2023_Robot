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
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class ElevatorGrabber extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax elevatorMotor;
  private CANSparkMax grabberMotor;

  private SparkMaxAbsoluteEncoder encoder;
  private DoubleSolenoid solenoid;

  private DigitalInput upperLimitSwitch;
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

    encoder = elevatorMotor.getAbsoluteEncoder(Type.kDutyCycle);
    encoder.setPositionConversionFactor(Constants.ElevatorGrabber.grabberMetersPerRotation);
    encoder.setVelocityConversionFactor(Constants.ElevatorGrabber.grabberMetersPerRotation);

    solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        Constants.ElevatorGrabber.inPneumaticChannel,
        Constants.ElevatorGrabber.outPneumaticChannel);

    upperLimitSwitch = new DigitalInput(Constants.ElevatorGrabber.upperLimitSwitchID);
    lowerLimitSwitch = new DigitalInput(Constants.ElevatorGrabber.lowerLimitSwitchID);

    velocityController = new PIDController(
        Constants.ElevatorGrabber.kP,
        Constants.ElevatorGrabber.kI,
        Constants.ElevatorGrabber.kD);

    feedforward = new SimpleMotorFeedforward(Constants.ElevatorGrabber.kS, Constants.ElevatorGrabber.kV);
  }


  private boolean atUpperLimit() {
    return upperLimitSwitch.get();
  }

  private boolean atLowerLimit() {
    return lowerLimitSwitch.get();
  }

  public void setElevatorMotorVolts(double volts) {
    elevatorMotor.setVoltage(volts);
  }

  public double getElevatorVelocityMetersPerSecond() {
    return encoder.getVelocity();
  }

  public void setElevatorMotorMetersPerSecond(double targetMetersPerSecond) {
    double currentMetersPerSecond = getElevatorVelocityMetersPerSecond();

    double pidOutput = velocityController.calculate(currentMetersPerSecond, targetMetersPerSecond);
    double feedforwardOutput = feedforward.calculate(currentMetersPerSecond);

    double totalOutput = pidOutput + feedforwardOutput;

    if(atLowerLimit() && totalOutput < 0)
      totalOutput = 0;
      
    if(atUpperLimit() && totalOutput > 0)
      totalOutput = 0;

    setElevatorMotorVolts(totalOutput);
  }

  public void extendPneumatics() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void retractPneumatics() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  public void setGrabMotor(double percentage) {
    grabberMotor.set(percentage);
  }

  public double getElevatorPositionMeters() {
    return encoder.getPosition();
  }

  public void zeroEncoder() {
    encoder.setZeroOffset(getElevatorPositionMeters());
  }

  @Override
  public void periodic() {
    if (atLowerLimit()) {
      zeroEncoder();
    }
  }
}
