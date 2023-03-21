// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pivot extends SubsystemBase {
  private DoubleSolenoid solenoid;

  public Pivot() {
    solenoid = new DoubleSolenoid(
      PneumaticsModuleType.REVPH,
      Constants.ElevatorGrabber.elevatorRetractPneumaticChannel,
      Constants.ElevatorGrabber.elevatorExtendPneumaticChannel
    );
  }

  /**
   * Extend elevator to score a game piece outside the frame perimeter
   */
  public void extendElevator() {
    solenoid.set(DoubleSolenoid.Value.kReverse);
  }

  /**
   * Bring elevator within the frame perimeter
   */
  public void retractElevator() {
    solenoid.set(DoubleSolenoid.Value.kForward);
  }

  public void toggleElevator() {
    solenoid.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
