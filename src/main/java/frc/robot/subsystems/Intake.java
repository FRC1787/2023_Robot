// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeIndexer.IndexerState;

public class IntakeIndex extends SubsystemBase {

  private final DoubleSolenoid intakeSolenoid;
  private final DoubleSolenoid indexerSolenoid;

  private CANSparkMax intakeMotor;
  private CANSparkMax conveyorMotor;
  private CANSparkMax clawMotor;
  private CANSparkMax leftIndexerMotor;
  private CANSparkMax rightIndexerMotor;

  private DigitalInput clawLimitSwitch;
  private RelativeEncoder clawMotorEncoder;

  private IndexerState indexerState;

  /** Creates a new IntakeIndex. */
  public IntakeIndex() {

    intakeSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.IntakeIndexer.intakeOutPneumaticsChannel,
      Constants.IntakeIndexer.intakeInPneumaticsChannel
    );

    indexerSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.IntakeIndexer.indexerOutPneumaticsChannel,
      Constants.IntakeIndexer.indexerInPneumaticsChannel
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

    clawMotor = new CANSparkMax(
      Constants.IntakeIndexer.clawMotorID,
      MotorType.kBrushless
    );

    leftIndexerMotor = new CANSparkMax(
      Constants.IntakeIndexer.leftIndexerMotorID,
      MotorType.kBrushless
    );
    rightIndexerMotor = new CANSparkMax(
      Constants.IntakeIndexer.rightIndexerMotorID,
      MotorType.kBrushless
    );

    clawLimitSwitch = new DigitalInput(Constants.IntakeIndexer.clawLimitSwitchID);

    clawMotorEncoder = clawMotor.getEncoder();
    clawMotorEncoder.setPositionConversionFactor(0);
  

    indexerState = IndexerState.cone;



    setMotorRampRates(1.0); //maybe delete??
  }

  private void setMotorInversions() {

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
   * @param voltage - Make this positive if you are trying to intake something.
   */
  public void setConveyorMotorVolts(double voltage) {
    conveyorMotor.set(voltage);
  }

  /** Stops all motors in this subsystem */
  public void stopIntakeMotors() {
    setIntakeMotorVolts(0);
    setConveyorMotorVolts(0);
  }



  /* INDEXER STUFF *//////////////////////////

  /**
   * Sets the voltage of the claw motor.
   * @param voltage - A positive value should move the claw towards the front of the robot.
   */
  public void setClawMotorVolts(double voltage) {
    clawMotor.set(voltage);
  }

  /**
   * Returns true if the claw is as far back as possible in the robot as possible,
   * analagous to it touching the limit switch.
   */
  public boolean isClawBack() {
    return clawLimitSwitch.get();
  }

  private void zeroClawEncoder() {
    clawMotorEncoder.setPosition(0);
  }

  /**
   * Returns true if the claw is as far forward in the robot as possible.
   */
  public boolean isClawForward() {
    return clawMotorEncoder.getPosition() > 15;
  }

  public void setCubeMode() {
    indexerState = IndexerState.cube;
  }

  public void setConeMode() {
    indexerState = IndexerState.cone;
  }

  public IndexerState getIndexerState() {
    return indexerState;
  }

  public void extendIndexerSolenoid() {
    indexerSolenoid.set(Value.kForward);
  }

  public void retractIndexerSolenoid() {
    indexerSolenoid.set(Value.kReverse);
  }

  /**
   * Sets voltage of indexer motors.
   * @param voltage - A positive value here should move the belts towards the front of the robot.
   */
  public void setIndexerMotors(double voltage) {
    leftIndexerMotor.set(voltage);
    rightIndexerMotor.set(voltage);
  }


  @Override
  public void periodic() {
    if (isClawBack())
      zeroClawEncoder();
  }
}
