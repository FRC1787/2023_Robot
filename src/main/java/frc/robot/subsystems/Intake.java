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
      Constants.IntakeIndexer.intakeExtendPneumaticsChannel,
      Constants.IntakeIndexer.intakeRetractPneumaticsChannel
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

    configureMotors();
  }

  private void configureMotors() {
    
    conveyorMotor.restoreFactoryDefaults();
    conveyorMotor.setInverted(true);

    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    

  }



  /* INTAKE/CONVEYOR STUFF */////////////////////////////

  /**
  * Extends the intake outwards to intake a game piece.
  */
  public void extendIntake() {
    intakeSolenoid.set(Value.kReverse);
  }

  /**
   * Retracts the intake into the robot.
   */
  public void retractIntake() {
    intakeSolenoid.set(Value.kForward);
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
   * @param voltage - A positive value moves an object on the conveyor towards the front of the robot.
   */
  public void setConveyorMotorVolts(double voltage) {
    conveyorMotor.set(voltage);
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
    setIntakeMotorVolts(0);
    setConveyorMotorVolts(0);
  }

  @Override
  public void periodic() {
  }
}
