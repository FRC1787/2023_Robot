// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intakeIndex.Claw;

public class MoveClawForward extends CommandBase {

  private Claw claw;
  private double clawMotorVoltage;
  private double motorRotations;

    /**
   * Moves claw towards back of robot.
   * @param percentage - percentage/speed of claw motor, the sign does not matter.
   * @param motorRotations - how many rotations of the motor to do (max should be 22, but this can be less if you want shorter travel)
   */
  public MoveClawForward(Claw claw, double clawMotorVoltage, double motorRotations) {
    addRequirements(claw);

    this.claw = claw;
    this.clawMotorVoltage = Math.abs(clawMotorVoltage);
    this.motorRotations = motorRotations;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    

    // clawMotorVoltage = SmartDashboard.getNumber("indexer claw forward voltage", 0.0);
    claw.setClawMotorVolts(clawMotorVoltage);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    claw.setClawMotorVolts(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return claw.getPosition() >= motorRotations;
  }
}
