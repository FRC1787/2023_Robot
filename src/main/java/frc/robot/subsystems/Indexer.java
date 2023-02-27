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

public class Indexer extends SubsystemBase {

  private final DoubleSolenoid indexerSolenoid;

  private CANSparkMax clawMotor;
  private CANSparkMax leftIndexerMotor;
  private CANSparkMax rightIndexerMotor;

  private DigitalInput clawLimitSwitch;
  private RelativeEncoder clawMotorEncoder;

  private IndexerState indexerState;

  public Indexer() {
    indexerSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.IntakeIndexer.indexerOutPneumaticsChannel,
      Constants.IntakeIndexer.indexerInPneumaticsChannel
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

    setMotorInversions();
    setMotorCurrentLimits();

  }

  private void setMotorInversions() {
    //TODO: set motor inversions
  }

  private void setMotorCurrentLimits() {
    clawMotor.setSmartCurrentLimit(30);
    //TODO: fix amp limit
  }


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
    // TODO: probably fix this value 
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

  /**
   * Makes the indexer walls parallel
   */
  public void extendIndexerSolenoid() {
    indexerSolenoid.set(Value.kForward);
  }

  /**
   * Makes the indexer walls angle inwards
   */
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
