// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GrabberPlacer extends SubsystemBase {
  private CANSparkMax motor;

  private LinearFilter ampFilter = LinearFilter.movingAverage(25);
  private double averageAmps;
  
  public GrabberPlacer() {
    motor = new CANSparkMax(
      Constants.ElevatorGrabber.grabberMotorID,
      MotorType.kBrushless
    );
    configureMotors();


    averageAmps = 0;

  }

  private void configureMotors() {
    motor.restoreFactoryDefaults();
    motor.setSmartCurrentLimit(50);
    motor.setInverted(false);
    motor.burnFlash();
  }

  /**
   * Sets voltage of the grab motor.
   * @param voltage - make this positive to intake a cone/outtake a cube,
   * while negative to outtake a cone/intake a cube 
   */
  public void setGrabMotorVolts(double voltage) {
    motor.setVoltage(voltage);
  }

  public void setGrabMotorAmpLimit(int ampLimit) {
    motor.setSmartCurrentLimit(ampLimit);
  }

  /**
   * Gets moving window average of amp readings for grabber motor
   * @return average amps over 8 readings
   */
  public double getGrabOutputAmps() {
    return averageAmps;
  }

  @Override
  public void periodic() {

    averageAmps = ampFilter.calculate(
      MathUtil.clamp(motor.getOutputCurrent(),
        0,
        50
      )
    );

    SmartDashboard.putNumber("amp reading for grabber", getGrabOutputAmps());
    SmartDashboard.putNumber("unfiltered amp limit", motor.getOutputCurrent());
  }
}
