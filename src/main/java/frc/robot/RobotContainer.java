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

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
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

  // CONTROLLERS
  public static final CommandXboxController controller = new CommandXboxController(0);
  public static final CommandXboxController backupController = new CommandXboxController(1);
  public static final CommandGenericHID buttonBoard = new CommandGenericHID(2);
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
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    autoChooser.setDefaultOption("1 cone + balance wire guard", "1 cone + balance wire guard");
    autoChooser.addOption("1 cone + balance middle", "1 cone + balance middle");
    autoChooser.addOption("1 cone + balance barrier", "1 cone + balance barrier");
    autoChooser.addOption("1 cone barrier", "1 cone barrier");
    autoChooser.addOption("1 cone middle", "1 cone middle");
    autoChooser.addOption("1 cone wire guard", "1 cone wire guard");
    autoChooser.addOption("gigachad auto barrier", "gigachad auto barrier");
    autoChooser.addOption("gigachad auto wire guard", "gigachad auto wire guard");
    autoChooser.addOption("goober", "goober");

    
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
    buttonBoard.button(16).and(controller.start()).onTrue(new IndexConeFull(intake, conveyor, indexerWalls, claw, elevator, pivot));

    inConeMode.onTrue(new SetGrabberMotor(grabberPlacer, 6, 100).withTimeout(0.15));

    // cube mode and cone mode toggles
    controller.a().onTrue(new InstantCommand(leds::setConeMode));
    controller.b().onTrue(new InstantCommand(leds::setCubeMode));
    buttonBoard.button(1).onTrue(new InstantCommand(leds::setConeMode));
    buttonBoard.button(2).onTrue(new InstantCommand(leds::setCubeMode));

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
        new MoveConveyor(conveyor, -8).withTimeout(0.70)
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
        new EjectGamePiece(intake, conveyor, indexerWalls, grabberPlacer, 12, 8, 8, -4)
      )
    ).onFalse(
      new InstantCommand(intake::retractIntake).andThen(
      new MoveElevatorToPosition(elevator, 0).asProxy())
    );
    //eject cube
    controller.leftTrigger().and(inConeMode.negate()).whileTrue(new EjectGamePiece(intake, conveyor, indexerWalls, grabberPlacer, 12, 8, 8, 6)).onFalse(new InstantCommand(intake::retractIntake));
    // hand off cone from indexer to grabber
    controller.rightBumper().onTrue(new PickUpCone(elevator, pivot, grabberPlacer, intake, conveyor, indexerWalls, claw));

    /* ALTERNATIVE SCORING CONTROLS TO TEST */
    //this.originalScoringBindings(); // operator moves the elevator and scores with a single button press
    //this.operatorConfirmBindings(); //operator moves elevator with one button, then confirms with another button
    this.driverConfirmBindings(); // driver gives the OK for the elevator to move to and score at the position being held by the operator
  }

  private void driverConfirmBindings() {
    //mid cone score
    buttonBoard.button(4).and(inConeMode).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.21)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, indexerWalls, true)));

    //high cone score
    buttonBoard.button(5).and(inConeMode).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.69)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, indexerWalls, true)));

    //mid cube score
    buttonBoard.button(4).and(inConeMode.negate()).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.21)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, indexerWalls, false)));

    //high cube score
    buttonBoard.button(5).and(inConeMode.negate()).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.7)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, indexerWalls, false)));
  
    //BACKUP CONTROLLER
    midScoreJoystick.and(inConeMode).and(controller.leftBumper())
    .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.21)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, indexerWalls, true)));

    //high cone score
    highScoreJoystick.and(inConeMode).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.69)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, indexerWalls, true)));

    //mid cube score
    midScoreJoystick.and(inConeMode.negate()).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.21)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, indexerWalls, false)));

    //high cube score
    highScoreJoystick.and(inConeMode.negate()).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevator, pivot, 1.7)).andThen(new ScoreGamePiece(elevator, pivot, grabberPlacer, indexerWalls, false)));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new InstantCommand(drivetrain::setGyroscope180)
    //  .andThen(new AutoRoutine("gigachad auto wire guard", drivetrain, vision, elevatorGrabber, indexer, intake, hatHack));

    //return new AutoRoutine("gigachad auto barrier", drivetrain, vision, grabberPlacer, elevator, pivot, indexerWalls, claw, intake, conveyor)
    //.andThen(new InstantCommand(drivetrain::setGyroscope180));

    // return new AutoRoutine("One Bumper Length Forward", drivetrain, vision, grabberPlacer, elevator, pivot, indexerWalls, claw, intake, conveyor);

     return new AutoRoutine(autoChooser.getSelected(), drivetrain, vision, grabberPlacer, elevator, pivot, indexerWalls, claw, intake, conveyor)
       .andThen(new InstantCommand(drivetrain::setGyroscope180));  
  }

// maybe sync button board LEDs?
/*
  public void syncCone() {
    simpleButtonBoard.setOutput(1, true);
    System.out.println("sync cone");
  }

  public void syncCube() {
    simpleButtonBoard.setOutput(2, true);
    System.out.println("sync cube");
  }
*/
}