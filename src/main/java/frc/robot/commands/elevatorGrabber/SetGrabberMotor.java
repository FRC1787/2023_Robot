// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.GrabberPlacer;

public class SetGrabberMotor extends CommandBase {

  GrabberPlacer grabberPlacer;
  double voltage;
  double ampLimit;

  /**
   * Sets the voltage of the elevator grabber motor, until the amp limit is reached.
   * @param grabberPlacer - The grabber place subsystem object.
   * @param voltage - Make this positive to intake a cone,
   * while negative to outtake a cone/intake or outtake a cube
   * @param ampLimit - When this amp limit is reached, the command ends.
   */
  public SetGrabberMotor(GrabberPlacer grabberPlacer, double voltage, double ampLimit) {
    addRequirements(grabberPlacer);
    
    this.grabberPlacer = grabberPlacer;
    this.voltage = voltage;
    this.ampLimit = ampLimit;
  
    // SmartDashboard.putNumber("elevator grabber motor voltage", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // voltage = SmartDashboard.getNumber("elevator grabber motor voltage", 0.0);
    grabberPlacer.setGrabMotorVolts(voltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    grabberPlacer.setGrabMotorVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return grabberPlacer.getGrabOutputAmps() > ampLimit;
  }
}
