// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevatorGrabber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.elevator.Elevator;

public class MoveElevatorToPosition extends CommandBase {

    Elevator elevator;
    double targetPositionMeters;
    TrapezoidProfile profile;
    Timer timer;
    double prevTimestamp;
    double currTimestamp;

    /**
     * Moves the elevator to a position in meters. The lowest position is marked as zero, and a higher position position is positive.
     * @param elevatorGrabber - elevatorGrabber subsystem object.
     * @param targetPositionMeters - target position in meters, positive is elevator extended.
     */
    public MoveElevatorToPosition(Elevator elevator, double targetPositionMeters) {
        addRequirements(elevator);

        this.elevator = elevator;
        this.targetPositionMeters = targetPositionMeters;
        timer = new Timer();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        profile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constants.ElevatorGrabber.elevatorMaxVelMetersPerSecond,
                Constants.ElevatorGrabber.elevatorMaxAccelMetersPerSecondSquared),
            new TrapezoidProfile.State(
                targetPositionMeters, 0.0
            ),
            new TrapezoidProfile.State(
                elevator.getElevatorPositionMeters(), elevator.getElevatorVelocityMetersPerSecond()
            )  
        );

        
        timer.reset();
        timer.start();
        System.out.println("New Elevator Target: " + targetPositionMeters);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // save info from the prev iteration before calculating for this iteration:
        prevTimestamp = currTimestamp;
        double prevVelocityCommand = profile.calculate(prevTimestamp).velocity;

        // get current commands for velocity and acceleration
        // from the motion profile
        currTimestamp = timer.get();
        double currVelocityCommand = profile.calculate(currTimestamp).velocity;
        double currAccelerationCommand = (currVelocityCommand - prevVelocityCommand) / (currTimestamp - prevTimestamp);
        currAccelerationCommand = 0; // Temporarily disable this!

        // incorporate position feedback!
        double measuredPosition = elevator.getElevatorPositionMeters();
        double desiredPosition = profile.calculate(currTimestamp).position;
        double positionError = desiredPosition - measuredPosition;
        double extraVelocityPerMeter = 8;  //analagous to I term of velocity controller

        currVelocityCommand += positionError * extraVelocityPerMeter;

        elevator.setElevatorMotorMetersPerSecond(
            currVelocityCommand, currAccelerationCommand);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        elevator.setElevatorMotorMetersPerSecond(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return profile.isFinished(timer.get()) && (Math.abs(elevator.getElevatorPositionMeters() - targetPositionMeters) <= 0.005);
    }
}