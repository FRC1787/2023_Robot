// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.elevatorGrabber.ElevatorIdle;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPosition;
import frc.robot.commands.elevatorGrabber.PickUpCone;
import frc.robot.commands.elevatorGrabber.SetGrabberMotor;
import frc.robot.commands.indexer.IndexConeFull;
import frc.robot.commands.indexer.MoveClawBack;
import frc.robot.commands.intake.EjectGamePiece;
import frc.robot.commands.intake.IntakeGamePieces;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.intakeIndex.Intake;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.subsystems.LEDs;

import java.time.Instant;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.GrabberPlacer;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.intakeIndex.Claw;
import frc.robot.subsystems.intakeIndex.IndexerWalls;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public List<PathPlannerTrajectory> pathGroup;

  // CONTROLLERS
  public static final CommandXboxController controller = new CommandXboxController(0);
  public static final CommandXboxController backupController = new CommandXboxController(1);
  public static final Joystick joystick = new Joystick(3);

  // SUBSYSTEMS
  final Drivetrain drivetrain = new Drivetrain();
  final Intake intake = new Intake();
  final Conveyor conveyor = new Conveyor();
  final IndexerWalls indexerWalls = new IndexerWalls();
  final Claw claw = new Claw();
  final Vision vision = new Vision();
  final Elevator elevator = new Elevator();
  final GrabberPlacer grabberPlacer = new GrabberPlacer();
  final Pivot pivot = new Pivot();
  final LEDs leds = new LEDs();


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();


    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, true));
    // Makes sure the claw is homed.
    claw.setDefaultCommand(new MoveClawBack(claw, 1));
    elevator.setDefaultCommand(new ElevatorIdle(elevator));
    indexerWalls.setDefaultCommand(new InstantCommand(indexerWalls::openIndexerWalls, indexerWalls));

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    controller.a().onTrue(
      new InstantCommand(intake::extendIntake).andThen(
        new MoveElevatorToPosition(elevator, 1.6)));

    controller.b().onTrue(
      new InstantCommand(intake::extendIntake).andThen(
        new MoveElevatorToPosition(elevator, 1.)));

    controller.x().onTrue(
      new MoveElevatorToPosition(elevator, 0));

    controller.y().onTrue(new InstantCommand(drivetrain::zeroYaw));
    controller.rightTrigger().whileTrue(new IntakeGamePieces(intake, conveyor, indexerWalls, pivot, 4, 0, 0));
    controller.povUp().whileTrue(new InstantCommand(() -> grabberPlacer.setGrabMotorVolts(3))).onFalse(new InstantCommand(() -> grabberPlacer.setGrabMotorVolts(0)));
    controller.povDown().whileTrue(new InstantCommand(() -> grabberPlacer.setGrabMotorVolts(-3))).onFalse(new InstantCommand(() -> grabberPlacer.setGrabMotorVolts(0)));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // we always start facing towards the alliance station, so our inital angle
    // should always be 180
    return null;
  }
}