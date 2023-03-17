// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.AutoRoutine;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.elevatorGrabber.ExtendElevatorToPosition;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPosition;
import frc.robot.commands.elevatorGrabber.PickUpCone;
import frc.robot.commands.elevatorGrabber.PickUpCube;
import frc.robot.commands.elevatorGrabber.ScoreGamePiece;
import frc.robot.commands.elevatorGrabber.SetGrabberMotorHatHack;
import frc.robot.commands.indexer.IndexConeFull;
import frc.robot.commands.indexer.MoveClawBack;
import frc.robot.commands.intake.EjectGamePiece;
import frc.robot.commands.intake.IntakeGamePieces;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.subsystems.CubeHatHack;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorGrabber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Vision;

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

  // SUBSYSTEMS 
  final Drivetrain drivetrain = new Drivetrain();
  final Intake intake = new Intake();
  final Indexer indexer = new Indexer();
  final Vision vision = new Vision();
  final ElevatorGrabber elevatorGrabber = new ElevatorGrabber();
  final CubeHatHack hatHack = new CubeHatHack();


  public final Trigger inConeMode = new Trigger(indexer::inConeMode); // TODO: do we need this?
  //public static final GenericHID simpleButtonBoard = new GenericHID(2); // alternate LED sync method?

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

    // stuff we're testing TODO: (maybe delete for competition?)
    //controller.a().whileTrue(new MoveElevatorToPositionSmartDashboard(elevatorGrabber));
    //controller.b().whileTrue(new MoveElevatorToPosition(elevatorGrabber, 0));
    // controller.povLeft().whileTrue(new PickUpCone(elevatorGrabber, intake, indexer));
    // controller.povUp().whileTrue(new PickUpCube(intake, elevatorGrabber, indexer, hatHack));
    //controller.back().whileTrue(new AlignToTarget(drivetrain, vision, Constants.Vision.LimelightTarget.aprilTag));
    controller.back().whileTrue(new AutoBalance(drivetrain));
    buttonBoard.button(16).and(controller.start()).onTrue(new IndexConeFull(intake, indexer, elevatorGrabber));

    inConeMode.onTrue(new SetGrabberMotorHatHack(elevatorGrabber, 6, 100, hatHack).withTimeout(0.15));

    // cube mode and cone mode toggles
    controller.a().onTrue(new InstantCommand(indexer::setConeMode));
    controller.b().onTrue(new InstantCommand(indexer::setCubeMode));
    buttonBoard.button(1).onTrue(new InstantCommand(indexer::setConeMode));
    buttonBoard.button(2).onTrue(new InstantCommand(indexer::setCubeMode));
    //controller.a().onTrue((new InstantCommand(indexer::setConeMode)).andThen(new InstantCommand(this::syncCone)));
    //controller.b().onTrue((new InstantCommand(indexer::setCubeMode)).andThen(new InstantCommand(this::syncCube)));

    
    // drivetrain
    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, true));
    controller.y().onTrue(new InstantCommand(drivetrain::zeroYaw));

    // intake cone
    controller.rightTrigger().and(inConeMode)
      .whileTrue(
        new ParallelCommandGroup(
          new MoveClawBack(indexer, 3.6),
          new IntakeGamePieces(intake, indexer, elevatorGrabber, -4, -12, -6),
          new MoveElevatorToPosition(elevatorGrabber, 0))
      );

    // intake cube
    controller.rightTrigger().and(inConeMode.negate())
      .whileTrue(
        new ParallelCommandGroup(
          new MoveClawBack(indexer, 3.6),
          new IntakeGamePieces(intake, indexer, elevatorGrabber, -3, -5, -6),
          new MoveElevatorToPosition(elevatorGrabber, 0))
      );

    
    //index cone upon trigger release
    controller.rightTrigger().and(inConeMode)
      .onFalse(
        new MoveConveyor(intake, -8).withTimeout(0.70)
        .andThen(new IndexConeFull(intake, indexer, elevatorGrabber))
      );

    //get cube in grabber upon intake release
    controller.rightTrigger().and(inConeMode.negate())
      .onFalse(
        new WaitCommand(0.5).andThen(
        new PickUpCube(intake, elevatorGrabber, indexer, hatHack))
      );
    
    //eject cone
    controller.leftTrigger().and(inConeMode).whileTrue(new EjectGamePiece(intake, indexer, elevatorGrabber, 12, 8, 8, -6));
    //eject cube
    controller.leftTrigger().and(inConeMode.negate()).whileTrue(new EjectGamePiece(intake, indexer, elevatorGrabber, 12, 8, 8, 6));
    // hand off cone from indexer to grabber
    controller.rightBumper().onTrue(new PickUpCone(elevatorGrabber, intake, indexer));

    /* ALTERNATIVE SCORING CONTROLS TO TEST */
    //this.originalScoringBindings(); // operator moves the elevator and scores with a single button press
    //this.operatorConfirmBindings(); //operator moves elevator with one button, then confirms with another button
    this.driverConfirmBindings(); // driver gives the OK for the elevator to move to and score at the position being held by the operator
  }

  private void originalScoringBindings() {
    //mid cone score
    buttonBoard.button(4).and(inConeMode)
      .onTrue((new ExtendElevatorToPosition(elevatorGrabber, 1.21)).andThen(new ScoreGamePiece(elevatorGrabber, indexer, true)));

    //high cone score
    buttonBoard.button(5).and(inConeMode)
      .onTrue((new ExtendElevatorToPosition(elevatorGrabber, 1.69)).andThen(new ScoreGamePiece(elevatorGrabber, indexer, true)));

    //mid cube score
    buttonBoard.button(4).and(inConeMode.negate())
      .onTrue((new ExtendElevatorToPosition(elevatorGrabber, 1.21)).andThen(new ScoreGamePiece(elevatorGrabber, indexer, false)));

    //high cube score
    buttonBoard.button(5).and(inConeMode.negate())
      .onTrue((new ExtendElevatorToPosition(elevatorGrabber, 1.7)).andThen(new ScoreGamePiece(elevatorGrabber, indexer, false)));
  }

  private void operatorConfirmBindings() {
    //mid cone score
    buttonBoard.button(4).and(inConeMode)
      .onTrue(new ExtendElevatorToPosition(elevatorGrabber, 1.21));
    buttonBoard.button(4).and(buttonBoard.button(1))
      .onTrue(new ScoreGamePiece(elevatorGrabber, indexer, true));

    //high cone score
    buttonBoard.button(5).and(inConeMode)
      .onTrue(new ExtendElevatorToPosition(elevatorGrabber, 1.69));
    buttonBoard.button(5).and(buttonBoard.button(1))
      .onTrue(new ScoreGamePiece(elevatorGrabber, indexer, true));

    // TODO: low cone score (simply ejecting with left trigger isn't reliable enough?)

    // cube is separate because possible different elevator heights and maybe reversed scoregamepiece
    //mid cube score
    buttonBoard.button(4).and(inConeMode.negate())
      .onTrue(new ExtendElevatorToPosition(elevatorGrabber, 1.21));
    buttonBoard.button(4).and(buttonBoard.button(2))
      .onTrue(new ScoreGamePiece(elevatorGrabber, indexer, false));

    //high cube score
    buttonBoard.button(5).and(inConeMode.negate())
      .onTrue(new ExtendElevatorToPosition(elevatorGrabber, 1.7));
    buttonBoard.button(5).and(buttonBoard.button(2))
      .onTrue(new ScoreGamePiece(elevatorGrabber, indexer, false));
  }

  private void driverConfirmBindings() {
    //mid cone score
    buttonBoard.button(4).and(inConeMode).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevatorGrabber, 1.21)).andThen(new ScoreGamePiece(elevatorGrabber, indexer, true)));

    //high cone score
    buttonBoard.button(5).and(inConeMode).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevatorGrabber, 1.69)).andThen(new ScoreGamePiece(elevatorGrabber, indexer, true)));

    //mid cube score
    buttonBoard.button(4).and(inConeMode.negate()).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevatorGrabber, 1.21)).andThen(new ScoreGamePiece(elevatorGrabber, indexer, false)));

    //high cube score
    buttonBoard.button(5).and(inConeMode.negate()).and(controller.leftBumper())
      .onTrue((new ExtendElevatorToPosition(elevatorGrabber, 1.7)).andThen(new ScoreGamePiece(elevatorGrabber, indexer, false)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand(drivetrain::setGyroscope180)
      .andThen(new AutoRoutine(autoChooser.getSelected(), drivetrain, vision, elevatorGrabber, indexer, intake, hatHack));

    // return new AutoRoutine(autoChooser.getSelected(), drivetrain, vision, elevatorGrabber, indexer, intake)
      // .andThen(new InstantCommand(drivetrain::setGyroscope180));  
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