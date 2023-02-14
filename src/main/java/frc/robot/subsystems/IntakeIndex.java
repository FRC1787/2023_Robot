// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeIndex extends SubsystemBase {

  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(
    // REVPH declares we are using REV's Pneumatic Hub.
    PneumaticsModuleType.REVPH,
    // The two channels are the Pneumatic Hub ports the solenoid is wired to.
    Constants.IntakeIndexer.intakeOutPneumaticsChannel,
    Constants.IntakeIndexer.intakeInPneumaticsChannel
  );

  private CANSparkMax intakeMotor = new CANSparkMax(
    // CAN ID of the Spark MAX
    Constants.IntakeIndexer.intakeMotorID, 
    MotorType.kBrushless
  );

  private CANSparkMax conveyorMotor = new CANSparkMax(
    Constants.IntakeIndexer.conveyorMotorID,
    MotorType.kBrushless
  );

  /** Creates a new IntakeIndex. */
  public IntakeIndex() {
    intakeMotor.setSmartCurrentLimit(60);
    conveyorMotor.setSmartCurrentLimit(60);
  }

  /**
  * Sets the value of the intake solenoid
  * 
  * <p>
  * To extend the pneumatic cylinder, use {@code kForward} for the {@code value} parameter, for example: 
  * {@code setPiston(Value.kForward);}
  * <p>
  * To retract the pneumatic cylinder, use {@code kReverse} for the {@code value} parameter, for example:
  * {@code setPiston(Value.kReverse);}
  * 
  * @param value - {@code Value.kForward} or {@code Value.kReverse}
  */
  public void setIntake(DoubleSolenoid.Value directionValue) {
    intakeSolenoid.set(directionValue);
  }

  /**
   * Read the current value of the intake solenoid.
   * @return The current value of the intake solenoid.
   */
  public DoubleSolenoid.Value getIntakeDirection() {
    return intakeSolenoid.get();
  }

  /**
   * Sets the speed of the intake motor.
   * <p>
   * To set the speed of the motor, input a double between -1.0 and 1.0. For example,
   * {@code setIntakeMotor(0.5);}
   * @param speed Speed to set for the motor. Value [-1.0, 1.0].
   */
  public void setIntakeMotor(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Sets the speed of the conveyor motor.
   * <p>
   * To set the speed of the motor, input a double between -1.0 and 1.0. For example,
   * {@code setConveyorMotor(0.5);}
   * @param speed Speed to set for the motor. Value [-1.0, 1.0].
   */
  public void setConveyorMotor(double speed) {
    conveyorMotor.set(speed);
  }

  /** Stops all motors in this subsystem */
  public void stopAllMotors() {
    this.setIntakeMotor(0);
    this.setConveyorMotor(0);
  }

  /**
   * Sets the open loop ramp rate for this subsystem's motors.
   * <p>
   * This is the maximum rate at which the motor controllers' outputs are allowed to change.
   * @param rate Time in seconds to go from 0 to full throttle.
   */
  public void setRampRate(double rate) {
    intakeMotor.setOpenLoopRampRate(rate);
    conveyorMotor.setOpenLoopRampRate(rate);
  }

  @Override
  public void periodic() {}
}
