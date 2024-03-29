// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeIndex;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Conveyor extends SubsystemBase {

  private CANSparkMax conveyorMotor;
  
  ShuffleboardTab tab;
  GenericEntry inputVolts;

  public Conveyor() {

    tab = Shuffleboard.getTab("rpmvolts");
    inputVolts = Shuffleboard.getTab("rpmvolts")
      .add("input volts", 1)
      .getEntry();

    conveyorMotor = new CANSparkMax(
      Constants.IntakeIndexer.conveyorMotorID,
      MotorType.kBrushless
    );

    configureMotors();
  }

  private void configureMotors() {
    
    conveyorMotor.restoreFactoryDefaults();
    conveyorMotor.setInverted(true);
    if (conveyorMotor.getInverted() == false) {
      conveyorMotor.setInverted(true);
    }
    conveyorMotor.setSmartCurrentLimit(60);
    conveyorMotor.burnFlash();
  }



  /* INTAKE/CONVEYOR STUFF */////////////////////////////

  /**
   * Sets the voltage of the conveyor motor.
   * @param voltage - A positive value moves an object on the conveyor towards the front of the robot.
   */
  public void setConveyorMotorVolts(double voltage) {
    conveyorMotor.setVoltage(voltage);
  }

  /**
   * Get the direction of the conveyor
   * @return - A positive value means the conveyor is moving towards the front of the robot
   */
  public double getConveyorMotorDirection() {
    return Math.signum(conveyorMotor.get());
  }

  /** Stops all motors in this subsystem */
  public void stopIntakeMotors() {
    setConveyorMotorVolts(0);
  }

  public double getInputVolts() {
    return inputVolts.getDouble(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("conveyor velocity", conveyorMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Conveyor Amps", conveyorMotor.getOutputCurrent());
  }
}
