// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  private Spark blinkin;

  private IndexerState indexerState;

  private boolean hasBeenHomed;

  public Indexer() {
    indexerSolenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.IntakeIndexer.indexerOpenPneumaticsChannel,
      Constants.IntakeIndexer.indexerClosePneumaticsChannel
    );
    
    clawMotor = new CANSparkMax(
      Constants.IntakeIndexer.clawMotorID,
      MotorType.kBrushless
    );

    clawMotorEncoder = clawMotor.getEncoder();
    
    leftIndexerMotor = new CANSparkMax(
      Constants.IntakeIndexer.leftIndexerMotorID,
      MotorType.kBrushless
    );
    rightIndexerMotor = new CANSparkMax(
      Constants.IntakeIndexer.rightIndexerMotorID,
      MotorType.kBrushless
    );

    configureMotors();
    
    blinkin = new Spark(9);

    clawLimitSwitch = new DigitalInput(Constants.IntakeIndexer.clawLimitSwitchID);

    indexerState = IndexerState.cone;

    hasBeenHomed = false;
  }

  private void configureMotors() {
    leftIndexerMotor.restoreFactoryDefaults();
    leftIndexerMotor.setInverted(true);
    leftIndexerMotor.setSmartCurrentLimit(50); // TODO: double check this, if it's a neo550, this should be 20-40 amps.
    leftIndexerMotor.burnFlash();
    
    rightIndexerMotor.restoreFactoryDefaults();
    rightIndexerMotor.setInverted(false);
    rightIndexerMotor.setSmartCurrentLimit(50); // TODO: double check this, if it's a neo550, this should be 20-40 amps.
    rightIndexerMotor.burnFlash();

    clawMotor.restoreFactoryDefaults();
    clawMotor.setSmartCurrentLimit(20);
    clawMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
    clawMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
    clawMotor.burnFlash();
  }


  /**
   * Sets the voltage of the claw motor.
   * @param voltage - A positive value should move the claw towards the front of the robot.
   */
  public void setClawMotorVolts(double voltage) {
    if (isClawBack() && voltage < 0)
      clawMotor.setVoltage(0);
    else if (isClawForward() && voltage > 0)
      clawMotor.setVoltage(0);
    else 
      clawMotor.setVoltage(voltage);

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
   * Returns true if the claw is as far forward in the robot as possible (24 rotations of the claw motor)
   */
  public boolean isClawForward() {
    return clawMotorEncoder.getPosition() > 22;
  }

  public void setCubeMode() {
    indexerState = IndexerState.cube;
  }

  public void setConeMode() {
    indexerState = IndexerState.cone;
  }
  
  public boolean inConeMode() {
    return indexerState == IndexerState.cone;
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

  public double getIndexerDirection() {
    return Math.signum(leftIndexerMotor.get());
  }

  public void setLEDsYellow() {
    blinkin.set(0.69);
  }

  public void setLEDsPurple() {
    blinkin.set(0.91);
  }


  @Override
  public void periodic() {
    
    //update LEDs
    if (inConeMode()) {
      setLEDsYellow();
    }
    else {
      setLEDsPurple();
    }

    if (!hasBeenHomed) {
      if (!isClawBack()) {
        this.setClawMotorVolts(-1.0);
      }
      else {
        // once claw has been homed, this sends
        // one more command to make sure we're not backdriving.
        hasBeenHomed = true;
        this.setClawMotorVolts(0);
        System.out.println("done with homeing, this should never happen again!!!AAAAAAA");
      }
    }
    

    if (isClawBack()) {
      zeroClawEncoder();
    }

    

    SmartDashboard.putBoolean("in cone mode", inConeMode());
    SmartDashboard.putNumber("claw distance", clawMotorEncoder.getPosition());
    SmartDashboard.putBoolean("claw is back", isClawBack());
  }
}
