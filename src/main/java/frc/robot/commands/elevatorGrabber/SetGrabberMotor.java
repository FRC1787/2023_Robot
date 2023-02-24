// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorGrabber;

public class SetGrabberMotor extends CommandBase {

  ElevatorGrabber elevatorGrabber;
  double voltage;

  /**
   * Sets the voltage of the elevator grabber motor.
   * @param elevatorGrabber - The elevator grabber subsystem object.
   * @param voltage - Make this positive to intake a cone/outtake a cube,
   * while negative to outtake a cone/intake a cube 
   */
  public SetGrabberMotor(ElevatorGrabber elevatorGrabber, double voltage) {
    addRequirements(elevatorGrabber);

    this.elevatorGrabber = elevatorGrabber;
    this.voltage = voltage;
  
    SmartDashboard.putNumber("elevator grabber motor voltage", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    voltage = SmartDashboard.getNumber("elevator grabber motor voltage", 0.0);
    elevatorGrabber.setGrabMotorVolts(voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorGrabber.setGrabMotorVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
