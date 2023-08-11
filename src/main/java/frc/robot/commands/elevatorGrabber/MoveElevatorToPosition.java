// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorToPosition extends CommandBase {

    Elevator elevator;
    double targetPositionMeters;

    /**
     * Moves the elevator to a position in meters. The lowest position is marked as zero, and a higher position position is positive.
     * @param elevatorGrabber - elevatorGrabber subsystem object.
     * @param targetPositionMeters - target position in meters, positive is elevator extended.
     */
    public MoveElevatorToPosition(Elevator elevator, double targetPositionMeters) {
        addRequirements(elevator);

        this.elevator = elevator;
        this.targetPositionMeters = targetPositionMeters;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      elevator.moveElevatorToPosition(targetPositionMeters);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        //THIS CODE HAS BEEN MOVED TO Elevator.java
        //LEAVING THIS HERE COMMENTED FOR REFERENCE

        // // Saves info from the previous iteration before calculating for this iteration.
        // prevTimestamp = currTimestamp;
        // double prevVelocityCommand = profile.calculate(prevTimestamp).velocity;

        // // Gets current commands for velocity and acceleration from the motion profile.
        // currTimestamp = timer.get();
        // double currVelocityCommand = profile.calculate(currTimestamp).velocity;
        // double currAccelerationCommand = (currVelocityCommand - prevVelocityCommand) / (currTimestamp - prevTimestamp);
        // currAccelerationCommand = 0; // Temporarily disable this!

        // // Incorporate position feedback!
        // double measuredPosition = elevator.getElevatorPositionMeters();
        // double desiredPosition = profile.calculate(currTimestamp).position;
        // double positionError = desiredPosition - measuredPosition;
        // // Analagous to I term of velocity controller.
        // double extraVelocityPerMeter = 8;

        // currVelocityCommand += positionError * extraVelocityPerMeter;

        // elevator.setElevatorMotorMetersPerSecond(
        //     currVelocityCommand, currAccelerationCommand);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return !elevator.isMovingToTarget;
    }
}