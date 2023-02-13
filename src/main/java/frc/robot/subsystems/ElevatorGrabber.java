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
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ElevatorGrabber extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private CANSparkMax elevatorMotor;
  private PWMSparkMax grabberMotor;

  private RelativeEncoder encoder;
  private DoubleSolenoid solenoid;

  private DigitalInput upperLimitSwitch;
  private DigitalInput lowerLimitSwitch;

  private PIDController velocityController;
  private SimpleMotorFeedforward feedforward;

  public ElevatorGrabber() {
    elevatorMotor = new CANSparkMax(
        Constants.ElevatorGrabberConstants.elevatorMotorID,
        MotorType.kBrushless);

    grabberMotor = new PWMSparkMax(
        Constants.ElevatorGrabberConstants.grabberMotorID);

    encoder = elevatorMotor.getEncoder();

    solenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM,
        Constants.ElevatorGrabberConstants.inPneumaticChannel,
        Constants.ElevatorGrabberConstants.outPneumaticChannel);

    upperLimitSwitch = new DigitalInput(Constants.ElevatorGrabberConstants.upperLimitSwitchID);
    lowerLimitSwitch = new DigitalInput(Constants.ElevatorGrabberConstants.lowerLimitSwitchID);

    velocityController = new PIDController(
        Constants.ElevatorGrabberConstants.kP,
        Constants.ElevatorGrabberConstants.kI,
        Constants.ElevatorGrabberConstants.kD);

    feedforward = new SimpleMotorFeedforward(Constants.ElevatorGrabberConstants.kS, Constants.ElevatorGrabberConstants.kV);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
