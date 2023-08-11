// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import frc.robot.Constants;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Elevator extends SubsystemBase {

  ElevatorIO io;
  ElevatorIOInputsAutoLogged inputs;

  // This PID controller is used to follow the trapezoidal motion profile generated by the MoveElevatorToPostion command.
  private PIDController velocityController;
  private ElevatorFeedforward feedforward;
  private TrapezoidProfile trapezoidProfile;
  private Timer timer;
  public boolean isMovingToTarget;
  private double targetPositionMeters;

  public double desiredVelocity;
  private boolean hasBeenHomed;

  private Mechanism2d mech;
  private MechanismRoot2d mechRoot;
  private MechanismLigament2d mechElevator;

  public Elevator(ElevatorIO io) {

    this.io = io;
    inputs = new ElevatorIOInputsAutoLogged();

    // SysId recommends an depth of 5 - 10 samples per average (we picked 8 because 2^3).
    // encoder.setAverageDepth(8);
    // encoder.setMeasurementPeriod(0);



    velocityController = new PIDController(
        Constants.ElevatorGrabber.kPVoltsPerMeterPerSecond,
        Constants.ElevatorGrabber.kIVoltsPerMeter,
        Constants.ElevatorGrabber.kDVoltsPerMeterPerSecondSquared);

    feedforward = new ElevatorFeedforward(
      Constants.ElevatorGrabber.kSVolts,
      Constants.ElevatorGrabber.kGVolts,
      Constants.ElevatorGrabber.kVVoltsPer_MeterPerSecond,
      Constants.ElevatorGrabber.kAVoltsPer_MeterPerSecondSquared);

    timer = new Timer();

    desiredVelocity = 0;
    hasBeenHomed = false;

    //mechanism2d stuff to visualize the elevator
    mech = new Mechanism2d(0.73, 1.5);
    mechRoot = mech.getRoot("root", 0.3, 0.3);
    mechElevator = mechRoot.append(new MechanismLigament2d("elevator", 0.15, 60));
  }

    /**
   * Extend elevator to score a game piece outside the frame perimeter
   */
  public void extendElevator() {
    io.extendElevator();
    mechElevator.setAngle(Rotation2d.fromDegrees(30));
  }

  /**
   * Bring elevator within the frame perimeter
   */
  public void retractElevator() {
    io.retractElevator();
    mechElevator.setAngle(Rotation2d.fromDegrees(60));
  }

  public boolean hasBeenHomed() {
    return hasBeenHomed;
  }

  /**
   * Sets velocity of the elevator
   * @param targetMetersPerSecond - Make this positive if you want to extend the elevator up.
   */
  public void setElevatorMotorMetersPerSecond(double targetMetersPerSecond, double targetMetersPerSecondSquared) {
    desiredVelocity = targetMetersPerSecond;
    
    double currentMetersPerSecond = inputs.elevatorVelocityMetersPerSecond;

    double pidOutput = velocityController.calculate(currentMetersPerSecond, targetMetersPerSecond);
    double feedforwardOutput = feedforward.calculate(targetMetersPerSecond, targetMetersPerSecondSquared);

    double totalOutput = pidOutput + feedforwardOutput;

    // Stops elevator at limits.
    if ((inputs.atLowerLimit && totalOutput < 0) || (inputs.atUpperLimit && totalOutput > 0))
      totalOutput = 0;

    SmartDashboard.putNumber("elevator voltage", totalOutput);

    io.setElevatorMotorVoltage(totalOutput);
  }


  //moves elevator to a position in meters (0 being all the way down)
  public void moveElevatorToPosition(double targetPositionMeters) {

    targetPositionMeters = MathUtil.clamp(targetPositionMeters, 0, 1.71);

    if (Math.abs(this.targetPositionMeters - targetPositionMeters) < 0.01) {
      return;
    };

    this.targetPositionMeters = targetPositionMeters;

    trapezoidProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
        Constants.ElevatorGrabber.elevatorMaxVelMetersPerSecond,
        Constants.ElevatorGrabber.elevatorMaxAccelMetersPerSecondSquared
      ),
      new TrapezoidProfile.State(
        targetPositionMeters, 0.0
      ),
      new TrapezoidProfile.State(
        inputs.elevatorPositionMeters, inputs.elevatorVelocityMetersPerSecond
      )
    );

    timer.reset();
    timer.start();

    isMovingToTarget = true;

    System.out.println("New Elevator Target: " + targetPositionMeters);
  }

  private void followTrapezoidProfile() {


    //holding position if no trapezoid profile is active
    if (!isMovingToTarget) {
      //TODO: THIS THING DOESN't WORK BECAUSE ELEVATOR STUFF IS ANNOYING. WORK ON CREATING NEW PID OBJECT THAT ALLOWS CUSTOM INTEGRAL INPUT SO THAT WE DON't HAVE TO USE SIMON's JANKY HOMEMADE IMPLEMENTATION
      setElevatorMotorMetersPerSecond(0, 0);
      return;
    }

    //end trapezoid profile if reached desired position
    if (trapezoidProfile.isFinished(timer.get()) && (Math.abs(inputs.elevatorPositionMeters - targetPositionMeters) <= 0.005)) {
      isMovingToTarget = false;
      return;
    }



    double curTimestamp = timer.get();
    double desiredVelocity = trapezoidProfile.calculate(curTimestamp).velocity;

    // Incorporate position feedback!
    double measuredPosition = inputs.elevatorPositionMeters;
    double desiredPosition = trapezoidProfile.calculate(curTimestamp).position;
    double positionError = desiredPosition - measuredPosition;
    // Analagous to I term of velocity controller.
    double extraVelocityPerMeter = 8;
    desiredVelocity += positionError*extraVelocityPerMeter;

    SmartDashboard.putNumber("elevator desired velocity", desiredVelocity);

    setElevatorMotorMetersPerSecond(desiredVelocity, 0);

  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);

    if (inputs.atLowerLimit) {
      io.zeroEncoder();
      hasBeenHomed = true;
    }

    followTrapezoidProfile();

    Logger.getInstance().processInputs("Elevator", inputs);

    mechElevator.setLength(0.15 + inputs.elevatorPositionMeters);
    Logger.getInstance().recordOutput("elevator/mech2d", mech);
    Logger.getInstance().recordOutput("elevator/isMovingToTarget", isMovingToTarget);
  }
}
