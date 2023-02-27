// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {

  private final DoubleSolenoid intakeSolenoid;

  private CANSparkMax intakeMotor;
  private CANSparkMax conveyorMotor;
  
  /** Creates a new IntakeIndex. */
  public Intake() {
    intakeSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.IntakeIndexer.intakeOutPneumaticsChannel,
      Constants.IntakeIndexer.intakeInPneumaticsChannel
    );

    intakeMotor = new CANSparkMax(
      // CAN ID of the Spark MAX
      Constants.IntakeIndexer.intakeMotorID, 
      MotorType.kBrushless
    );

    conveyorMotor = new CANSparkMax(
      Constants.IntakeIndexer.conveyorMotorID,
      MotorType.kBrushless
    );

    setMotorRampRates(1.0); //maybe delete??
    setMotorInversions();
    setMotorCurrentLimits();
  }

  private void setMotorInversions() {
    // TODO: Set motor inversions?
  }

  private void setMotorCurrentLimits() {
    intakeMotor.setSmartCurrentLimit(60);
    conveyorMotor.setSmartCurrentLimit(60);
  }

  /**
   * Sets the open loop ramp rate for this subsystem's motors.
   * <p>
   * This is the maximum rate at which the motor controllers' outputs are allowed to change.
   * @param rate Time in seconds to go from 0 to full throttle.
   */
  public void setMotorRampRates(double rate) {
    intakeMotor.setOpenLoopRampRate(rate);
    conveyorMotor.setOpenLoopRampRate(rate);
  }


  /* INTAKE/CONVEYOR STUFF */////////////////////////////

  /**
  * Extends the intake solenoid.
  */
  public void extendIntake() {
    intakeSolenoid.set(Value.kForward);
  }

  /**
   * Retracts the intake solenoid.
   */
  public void retractIntake() {
    intakeSolenoid.set(Value.kReverse);
  }
  

  /**
   * Read the current value of the intake solenoid.
   * @return The current value of the intake solenoid.
   */
  public DoubleSolenoid.Value getIntakePosition() {
    return intakeSolenoid.get();
  }


  /**
   * Sets the voltage of the intake motor.
   * @param voltage - Make this positive if you are trying to intake something.
   */
  public void setIntakeMotorVolts(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  /**
   * Sets the voltage of the conveyor motor.
   * @param voltage - A positive value moves an object on the conveyor towards the back of the robot.
   */
  public void setConveyorMotorVolts(double voltage) {
    conveyorMotor.set(voltage);
  }

  /** Stops all motors in this subsystem */
  public void stopIntakeMotors() {
    setIntakeMotorVolts(0);
    setConveyorMotorVolts(0);
  }

  @Override
  public void periodic() {

  }
}
