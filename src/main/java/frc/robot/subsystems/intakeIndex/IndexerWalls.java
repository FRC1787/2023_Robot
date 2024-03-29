// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeIndex;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerWalls extends SubsystemBase {

  private final DoubleSolenoid indexerSolenoid;


  private CANSparkMax leftIndexerMotor;
  private CANSparkMax rightIndexerMotor;


  public IndexerWalls() {
    indexerSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.IntakeIndexer.indexerOpenPneumaticsChannel,
      Constants.IntakeIndexer.indexerClosePneumaticsChannel
    );
    

    
    leftIndexerMotor = new CANSparkMax(
      Constants.IntakeIndexer.leftIndexerMotorID,
      MotorType.kBrushless
    );
    rightIndexerMotor = new CANSparkMax(
      Constants.IntakeIndexer.rightIndexerMotorID,
      MotorType.kBrushless
    );    

    configureMotors();
  }

  private void configureMotors() {
    leftIndexerMotor.restoreFactoryDefaults();
    leftIndexerMotor.setInverted(true);
    leftIndexerMotor.setSmartCurrentLimit(20);
    leftIndexerMotor.burnFlash();
    
    rightIndexerMotor.restoreFactoryDefaults();
    rightIndexerMotor.setInverted(false);
    rightIndexerMotor.setSmartCurrentLimit(20);
    rightIndexerMotor.burnFlash();


  }

  public boolean isIndexerWallsOpen() {
    return indexerSolenoid.get() == Value.kReverse;
  }

  /**
   * Makes the indexer walls parallel
   */
  public void closeIndexerWalls() {
    indexerSolenoid.set(Value.kForward);
  }

  /**
   * Makes the indexer walls angle inwards
   */
  public void openIndexerWalls() {
    indexerSolenoid.set(Value.kReverse);
  }

  /**
   * Sets voltage of indexer motors.
   * @param voltage - A positive value here should move the belts towards the front of the robot.
   */
  public void setIndexerMotors(double voltage) {
    leftIndexerMotor.setVoltage(voltage);
    rightIndexerMotor.setVoltage(voltage);
  }

  public void stopIndexerMotors() {
    leftIndexerMotor.setVoltage(0);
    rightIndexerMotor.setVoltage(0);
  }

  public double getIndexerDirection() {
    return Math.signum(leftIndexerMotor.get());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("left side belt output amps", leftIndexerMotor.getOutputCurrent());
    SmartDashboard.putNumber("right side belt output amps", rightIndexerMotor.getOutputCurrent());
  }
}
