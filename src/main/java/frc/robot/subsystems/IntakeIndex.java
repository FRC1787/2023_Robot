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

    intakeMotor.setSmartCurrentLimit(60);
    conveyorMotor.setSmartCurrentLimit(60);

    setRampRate(1.0);
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
   * Sets the speed of the intake motor.
   * <p>
   * To set the speed of the motor, input a double between -1.0 and 1.0. For example,
   * {@code setIntakeMotor(0.5);}
   * @param speed Speed to set for the motor. Value [-1.0, 1.0].
   */
  public void setIntakeMotorPercentage(double percent) {
    intakeMotor.set(percent);
  }

  /**
   * Sets the speed of the conveyor motor.
   * <p>
   * To set the speed of the motor, input a double between -1.0 and 1.0. For example,
   * {@code setConveyorMotor(0.5);}
   * @param speed Speed to set for the motor. Value [-1.0, 1.0].
   */
  public void setConveyorMotorPercentage(double percent) {
    conveyorMotor.set(percent);
  }

  /** Stops all motors in this subsystem */
  public void stopIntakeMotors() {
    setIntakeMotorPercentage(0);
    setConveyorMotorPercentage(0);
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

  /* INDEXER STUFF *//////////////////////////

  /**
   * Sets the percentage of max voltage for the claw motor.
   * @param percentage Between -1.0 and 1.0, with positive being the claw moving towards the front of the robot.
   */
  public void setClawMotorPercentage(double percentage) {
    clawMotor.set(percentage);
  }

  private boolean getClawLimitSwitch() {
    return clawLimitSwitch.get();
  }

  private void zeroClawEncoder() {
    clawMotorEncoder.setPosition(0);
  }

  /**
   * Gets the position of the motor in rotations of the motor.
   */
  public double getClawMotorRotations() {
    return clawMotorEncoder.getPosition();
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


  @Override
  public void periodic() {
    if (getClawLimitSwitch())
      zeroClawEncoder();
  }
}
