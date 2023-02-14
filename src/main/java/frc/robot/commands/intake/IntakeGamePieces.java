// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeIndex;

public class IntakeGamePieces extends CommandBase {
  private IntakeIndex intakeIndex;
  private double intakeMotorSpeedRPM;
  private double conveyorMotorSpeedRPM;

  public IntakeGamePieces(IntakeIndex intakeIndex, double intakeMotorSpeedRPM, double conveyorMotorSpeedRPM) {
    addRequirements(intakeIndex);
    this.intakeIndex = intakeIndex;
    this.intakeMotorSpeedRPM = intakeMotorSpeedRPM;
    this.conveyorMotorSpeedRPM = conveyorMotorSpeedRPM;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeIndex.setIntake(Value.kForward);
    intakeIndex.setIntakeMotor(intakeMotorSpeedRPM);
    intakeIndex.setConveyorMotor(conveyorMotorSpeedRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeIndex.stopAllMotors();
    intakeIndex.setIntake(Value.kReverse);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
