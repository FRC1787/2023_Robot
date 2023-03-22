// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intakeIndex;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Claw extends SubsystemBase {

  private CANSparkMax clawMotor;

  private DigitalInput clawLimitSwitch;
  private RelativeEncoder clawMotorEncoder;

  private boolean hasBeenHomed;

  public Claw() {
    clawMotor = new CANSparkMax(
      Constants.IntakeIndexer.clawMotorID,
      MotorType.kBrushless
    );

    clawMotorEncoder = clawMotor.getEncoder();

    configureMotors();
  

    clawLimitSwitch = new DigitalInput(Constants.IntakeIndexer.clawLimitSwitchID);

    hasBeenHomed = false;
  }

  private void configureMotors() {
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

  @Override
  public void periodic() {

    //TODO: GET RID OF THIS UGLINESS
    if (!hasBeenHomed) {
      if (!isClawBack()) {
        this.setClawMotorVolts(-1.0);
      }
      else {
        // once claw has been homed, this sends
        // one more command to make sure we're not backdriving.
        hasBeenHomed = true;
        this.setClawMotorVolts(0);
      }
    }
    

    if (isClawBack()) {
      zeroClawEncoder();
    }

    SmartDashboard.putNumber("claw distance", clawMotorEncoder.getPosition());
    SmartDashboard.putBoolean("claw is back", isClawBack());
  }
}
