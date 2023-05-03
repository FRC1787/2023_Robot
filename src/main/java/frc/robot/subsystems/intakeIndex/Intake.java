// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeIndex;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final DoubleSolenoid intakeSolenoid;

  private CANSparkMax intakeMotor;


  /** Creates a new Intake. */
  public Intake() {
    intakeSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.IntakeIndexer.intakeExtendPneumaticsChannel,
      Constants.IntakeIndexer.intakeRetractPneumaticsChannel
    );

    intakeMotor = new CANSparkMax(
      // CAN ID of the SPARK MAX.
      Constants.IntakeIndexer.intakeMotorID, 
      MotorType.kBrushless
    );
  }

  public void configureMotors() {
    intakeMotor.restoreFactoryDefaults();
    intakeMotor.setInverted(false);
    // This leaves the motor spinning in order to further pull the cone in and ensure it doesn't get stuck on the intake.
    intakeMotor.setIdleMode(IdleMode.kBrake);
    intakeMotor.setSmartCurrentLimit(60);
    intakeMotor.burnFlash();
  }

  
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
   * @param voltage - Make this negative if you are trying to intake something.
   */
  public void setIntakeMotorVolts(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  public void stopIntakeMotor() {
    setIntakeMotorVolts(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // An error is not thrown when a SPARK MAX disconnected from the CAN network unless you are querrying values from it.
    SmartDashboard.putNumber("intakeNormalizedOutput", intakeMotor.getAppliedOutput());
  }
}
