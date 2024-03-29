// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeIndex.Conveyor;

public class MoveConveyor extends CommandBase {

  private Conveyor conveyor;
  private double conveyorVoltage;

  /**
   * Moves the conveyor at a given voltage
   * @param conveyor - intake subsystem object
   * @param conveyorVoltage - make this positive to move an object on the conveyor towards the front of the robot
   */
  public MoveConveyor(Conveyor conveyor, double conveyorVoltage) {
    addRequirements(conveyor);

    this.conveyor = conveyor;
    this.conveyorVoltage = conveyorVoltage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    conveyor.setConveyorMotorVolts(conveyorVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.setConveyorMotorVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
