// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeIndex.IndexerWalls;

public class MoveSideBelts extends CommandBase {
  

  IndexerWalls indexerWalls;
  double indexerSideBeltsMotorVoltage;

  /**
   * Moves side belts at a given voltage.
   * @param indexer - indexer subsystem object
   * @param indexerSideBeltsMotorVoltage - a positive value will move the belts towards the front of the robot.
   */
  public MoveSideBelts(IndexerWalls indexerWalls, double indexerSideBeltsMotorVoltage) {
    addRequirements(indexerWalls);

    this.indexerWalls = indexerWalls;
    this.indexerSideBeltsMotorVoltage = indexerSideBeltsMotorVoltage;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    indexerWalls.setIndexerMotors(indexerSideBeltsMotorVoltage);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexerWalls.setIndexerMotors(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
