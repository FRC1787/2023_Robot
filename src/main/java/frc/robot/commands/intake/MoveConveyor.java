// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveConveyor extends CommandBase {

  Intake intake;
  double conveyorVoltage;

  /**
   * Moves the conveyor at a given voltage
   * @param intake - intake subsystem object
   * @param conveyorVoltage - make this positive to move an object on the conveyor towards the front of the robot
   */
  public MoveConveyor(Intake intake, double conveyorVoltage) {
    addRequirements(intake);

    this.intake = intake;
    this.conveyorVoltage = conveyorVoltage;

    // SmartDashboard.putNumber("move conveyor motor voltage", 0.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // conveyorVoltage = SmartDashboard.getNumber("move conveyor motor voltage", 0.0);

    intake.setConveyorMotorVolts(conveyorVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setConveyorMotorVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
