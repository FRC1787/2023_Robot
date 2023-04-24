// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.AutoRoutine;
import frc.robot.commands.drivetrain.AlignWheelsToZero;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.elevatorGrabber.ElevatorIdle;
import frc.robot.commands.elevatorGrabber.ExtendElevatorToPosition;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPosition;
import frc.robot.commands.elevatorGrabber.PickUpCone;
import frc.robot.commands.elevatorGrabber.PickUpCube;
import frc.robot.commands.elevatorGrabber.ScoreGamePiece;
import frc.robot.commands.elevatorGrabber.SetGrabberMotor;
import frc.robot.commands.indexer.IndexConeFull;
import frc.robot.commands.indexer.MoveClawBack;
import frc.robot.commands.intake.BowlCube;
import frc.robot.commands.intake.EjectGamePiece;
import frc.robot.commands.intake.IntakeGamePieces;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.intakeIndex.Intake;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.subsystems.LEDs;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  public List<PathPlannerTrajectory> pathGroup;

  // CONTROLLERS
  public static final CommandXboxController controller = new CommandXboxController(0);
  public static final CommandXboxController backupController = new CommandXboxController(1);
  // public static final CommandGenericHID buttonBoard = new CommandGenericHID(2);
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

  public final Trigger inConeMode = new Trigger(leds::inConeMode);
  //public static final GenericHID simpleButtonBoard = new GenericHID(2); // alternate LED sync method?

  // BACKUP JOYSTICK
  private final Trigger highScoreJoystick = new JoystickButton(joystick, 7);
  private final Trigger midScoreJoystick = new JoystickButton(joystick, 9);
  private final Trigger coneModeJoystick = new JoystickButton(joystick, 11);
  private final Trigger cubeModeJoystick = new JoystickButton(joystick, 12);

  // AUTO
  public final SendableChooser<String> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    //for competition
    autoChooser.setDefaultOption("1 cone + balance wire guard", "1 cone + balance wire guard");
    autoChooser.addOption("1 cone + balance middle", "1 cone + balance middle");
    autoChooser.addOption("1 cone + balance barrier", "1 cone + balance barrier");
    autoChooser.addOption("1 cone barrier", "1 cone barrier");
    autoChooser.addOption("1 cone middle", "1 cone middle");
    autoChooser.addOption("1 cone wire guard", "1 cone wire guard");
    autoChooser.addOption("gigachad auto middle", "gigachad auto middle");
    // autoChooser.addOption("gigachad auto wire guard", "gigachad auto wire guard");
    autoChooser.addOption("cone + cube high barrier", "cone + cube high barrier");
    autoChooser.addOption("cone + cube high + balance barrier", "cone + cube high + balance barrier");
    autoChooser.addOption("1 cone + pickup wire guard", "1 cone + pickup wire guard");

    //dont use in game
    autoChooser.addOption("goober", "goober");
    autoChooser.addOption("not goober", "not goober");
    // autoChooser.addOption("TESTING DO NOT CHOOSE", "TESTING DO NOT CHOOSE");
    autoChooser.addOption("One Bumper Length Forward", "One Bumper Length Forward");
    // autoChooser.addOption("One Bumper Length Backward", "One Bumper Length Backward");
    // autoChooser.addOption("One Bumper Length Backward Plus Flip", "One Bumper Length Backward Plus Flip");
    // autoChooser.addOption("Maybe 3 Piece Barrier", "Maybe 3 Piece Barrier");
    // autoChooser.addOption("Maybe 3 Piece Gigachad", "Maybe 3 Piece Gigachad");


    
    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, true));
    grabberPlacer.setDefaultCommand(new SetGrabberMotor(grabberPlacer, 0.5, 100));
    claw.setDefaultCommand(new MoveClawBack(claw, 1)); // makes sure the claw is homed
    elevator.setDefaultCommand(new ElevatorIdle(elevator));
    indexerWalls.setDefaultCommand(new InstantCommand(indexerWalls::openIndexerWalls, indexerWalls));

    SmartDashboard.putData(autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // toggles robot oriented with right bumper
    // controller.rightBumper().whileTrue(new JoystickDrive(drivetrain, false));
    // controller.rightBumper().whileFalse(new JoystickDrive(drivetrain, true));

    // stuff we're testing
    //controller.a().whileTrue(new MoveElevatorToPositionSmartDashboard(elevatorGrabber));
    //controller.b().whileTrue(new MoveElevatorToPosition(elevatorGrabber, 0));
    // controller.povLeft().whileTrue(new PickUpCone(elevatorGrabber, intake, indexer));
    // controller.povUp().whileTrue(new PickUpCube(intake, elevatorGrabber, indexer, hatHack));
    //controller.back().whileTrue(new AlignToTarget(drivetrain, vision, Constants.Vision.LimelightTarget.aprilTag));
    controller.povLeft().whileTrue(new BowlCube(intake, conveyor, indexerWalls, grabberPlacer, 6, 4, 4, 0).withTimeout(3));
    controller.povRight().whileTrue(new AlignWheelsToZero(drivetrain));
    controller.back().whileTrue(new AutoBalance(drivetrain));
    // buttonBoard.button(16).and(controller.start()).onTrue(new IndexConeFull(intake, conveyor, indexerWalls, claw, elevator, pivot));

    inConeMode.onTrue(new SetGrabberMotor(grabberPlacer, 6, 100).withTimeout(0.15));

    // cube mode and cone mode toggles
    controller.a().onTrue(new InstantCommand(leds::setConeMode));
    controller.b().onTrue(new InstantCommand(leds::setCubeMode));
    // buttonBoard.button(1).onTrue(new InstantCommand(leds::setConeMode));
    // buttonBoard.button(2).onTrue(new InstantCommand(leds::setCubeMode));

    coneModeJoystick.onTrue(new InstantCommand(leds::setConeMode));
    cubeModeJoystick.onTrue(new InstantCommand(leds::setCubeMode));

    //controller.a().onTrue((new InstantCommand(indexer::setConeMode)).andThen(new InstantCommand(this::syncCone)));
    //controller.b().onTrue((new InstantCommand(indexer::setCubeMode)).andThen(new InstantCommand(this::syncCube)));

    // drivetrain
    controller.y().onTrue(new InstantCommand(drivetrain::zeroYaw));

    // intake cone
    controller.rightTrigger().and(inConeMode)
      .whileTrue(
        new ParallelCommandGroup(
          new MoveClawBack(claw, 3.6),
          new IntakeGamePieces(intake, conveyor, indexerWalls, pivot, -4, -12, -6),
          new MoveElevatorToPosition(elevator, 0).asProxy())
      );

    // intake cube
    controller.rightTrigger().and(inConeMode.negate())
      .whileTrue(
        new ParallelCommandGroup(
          new MoveClawBack(claw, 3.6),
          new IntakeGamePieces(intake, conveyor, indexerWalls, pivot, -3, -5, -6),
          new MoveElevatorToPosition(elevator, 0).asProxy())
      );

    
    //index cone upon trigger release
    controller.rightTrigger().and(inConeMode)
      .onFalse(
        new MoveConveyor(conveyor, -9).withTimeout(0.50) // <- if something breaks with the intake to index sequence then this is why
        .andThen(new IndexConeFull(intake, conveyor, indexerWalls, claw, elevator, pivot))
      );

    //get cube in grabber upon intake release
    controller.rightTrigger().and(inConeMode.negate())
      .onFalse(
        new WaitCommand(0.5).andThen(
        new PickUpCube(intake, conveyor, elevator, pivot, grabberPlacer, indexerWalls))
      );
    
    //eject cone
    controller.leftTrigger().and(inConeMode).whileTrue(
      new ParallelCommandGroup(
        new MoveElevatorToPosition(elevator, .4).asProxy(),
        new EjectGamePiece(intake, pivot, conveyor, indexerWalls, grabberPlacer, 12, 8, 8, -4)
      )
    ).onFalse(
      new InstantCommand(intake::retractIntake).andThen(
      new MoveElevatorToPosition(elevator, 0).asProxy())
    );

    //eject cube
    controller.leftTrigger().and(inConeMode.negate()).whileTrue(
      new BowlCube(intake, conveyor, indexerWalls, grabberPlacer, 6.75, 2.5, 2.5, 4)
    ).onFalse(
      new MoveElevatorToPosition(elevator, 0).asProxy()
    );
    
    // hand off cone from indexer to grabber
    controller.rightBumper().and(inConeMode).onTrue(new PickUpCone(elevator, pivot, grabberPlacer, intake, conveyor, indexerWalls, claw));
    controller.rightBumper().and(inConeMode.negate()).onTrue(new PickUpCube(intake, conveyor, elevator, pivot, grabberPlacer, indexerWalls));
    /* ALTERNATIVE SCORING CONTROLS TO TEST */
    this.driverConfirmBindings(); // driver gives the OK for the elevator to move to and score at the position being held by the operator
  }

  private void driverConfirmBindings() {  
    //BACKUP CONTROLLER
    midScoreJoystick.and(inConeMode).and(controller.leftBumper())
    .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.21)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, true)));

    //high cone score
    highScoreJoystick.and(inConeMode).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.69)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, true)));

    //mid cube score
    midScoreJoystick.and(inConeMode.negate()).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.18)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, false)));

    //high cube score
    highScoreJoystick.and(inConeMode.negate()).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.7)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, false)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // we always start facing towards the alliance station, so our inital angle should always be 180
    return new InstantCommand(drivetrain::setGyroscope180).andThen(
    new AutoRoutine(pathGroup, drivetrain, vision, grabberPlacer, elevator, pivot, indexerWalls, claw, intake, conveyor));
  }
}