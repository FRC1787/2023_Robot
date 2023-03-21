// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class ElevatorIdle extends CommandBase {
  /** Creates a new ElevatorIdle. */

  private Elevator elevator;

  public ElevatorIdle(Elevator elevator) {
    addRequirements(elevator);

    this.elevator=elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!elevator.hasBeenHomed()) {
      elevator.setElevatorMotorVolts(-0.2);
    }
    else {
      elevator.setElevatorMotorMetersPerSecond(0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setElevatorMotorMetersPerSecond(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
