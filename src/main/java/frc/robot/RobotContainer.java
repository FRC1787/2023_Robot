// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.autonomous.AutoBalance;
import frc.robot.commands.autonomous.AutoRoutine;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.elevatorGrabber.ExtendElevatorToPosition;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPosition;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPositionSmartDashboard;
import frc.robot.commands.elevatorGrabber.PickUpCone;
import frc.robot.commands.elevatorGrabber.PickUpCube;
import frc.robot.commands.elevatorGrabber.RetractAndHomeElevator;
import frc.robot.commands.elevatorGrabber.ScoreGamePiece;
import frc.robot.commands.elevatorGrabber.SetGrabberMotor;
import frc.robot.commands.indexer.IndexConeFull;
import frc.robot.commands.intake.EjectGamePiece;
import frc.robot.commands.intake.IntakeGamePieces;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorGrabber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.LED;
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
  public static final Joystick joystick = new Joystick(3);
  public static final CommandXboxController backupController = new CommandXboxController(1);
  public static final CommandGenericHID buttonBoard = new CommandGenericHID(2);

  // SUBSYSTEMS 
  final Drivetrain drivetrain = new Drivetrain();
  final Intake intake = new Intake();
  final Indexer indexer = new Indexer();
  final Vision vision = new Vision();
  final ElevatorGrabber elevatorGrabber = new ElevatorGrabber();
  final LED led = new LED();

  // testing
  private final JoystickButton intakeOut = new JoystickButton(joystick, 8);
  private final JoystickButton intakeIn = new JoystickButton(joystick, 7);
  private final JoystickButton extendElevator = new JoystickButton(joystick, 10);
  private final JoystickButton retractElevator = new JoystickButton(joystick, 9);
  private final JoystickButton sideBeltsIn = new JoystickButton(joystick, 12);
  private final JoystickButton sideBeltsOut = new JoystickButton(joystick, 11);

  // AUTO
  private final SendableChooser<String> autoChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    led.setOrange();

    autoChooser.setDefaultOption("Full Path", "FullPath");
    autoChooser.addOption("Curve 180", "curve180");

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

    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, true));

    controller.a().whileTrue(new MoveElevatorToPositionSmartDashboard(elevatorGrabber));
    controller.b().whileTrue(new MoveElevatorToPosition(elevatorGrabber, 0));

    controller.povLeft().whileTrue(new PickUpCone(elevatorGrabber, intake, indexer));
    controller.povUp().whileTrue(new PickUpCube(intake, elevatorGrabber, indexer));

    controller.y().onTrue(new InstantCommand(drivetrain::zeroYaw));

    //intake
    controller.rightTrigger()
      .whileTrue(
        new ParallelCommandGroup(
          new IntakeGamePieces(intake, indexer, elevatorGrabber, -4, -12, -6),
          new MoveElevatorToPosition(elevatorGrabber, 0))
      );
    
    //index cone upon trigger release
    controller.rightTrigger().and(buttonBoard.button(16))
      .onFalse(
        new MoveConveyor(intake, -8).withTimeout(0.70)
        .andThen(new IndexConeFull(intake, indexer, elevatorGrabber))
      );

    //get cube in grabber upon intake release
    controller.rightTrigger().and(buttonBoard.button(16).negate())
      .onFalse(
        new PickUpCube(intake, elevatorGrabber, indexer)
      );
    
    //eject cone
    controller.leftTrigger().and(buttonBoard.button(16)).whileTrue(new EjectGamePiece(intake, indexer, elevatorGrabber, 12, 8, 8, -6));
    //eject cube
    controller.leftTrigger().and(buttonBoard.button(16).negate()).whileTrue(new EjectGamePiece(intake, indexer, elevatorGrabber, 12, 8, 8, 6));

    //pick up cone
    controller.rightBumper().onTrue(new PickUpCone(elevatorGrabber, intake, indexer));
    
    //controller.back().whileTrue(new AlignToTarget(drivetrain, vision, Constants.Vision.LimelightTarget.aprilTag));

    controller.back().whileTrue(new AutoBalance(drivetrain));

    //pneumatics stuff delete later
    intakeIn.onTrue(new InstantCommand(intake::retractIntake));
    intakeOut.onTrue(new InstantCommand(intake::extendIntake));

    extendElevator.onTrue(new InstantCommand(elevatorGrabber::extendElevator));
    retractElevator.onTrue(new InstantCommand(elevatorGrabber::retractElevator));

    sideBeltsIn.onTrue(new InstantCommand(indexer::closeIndexerWalls));
    sideBeltsOut.onTrue(new InstantCommand(indexer::openIndexerWalls));

    buttonBoard.button(16).and(controller.start()).onTrue(new IndexConeFull(intake, indexer, elevatorGrabber));

    //mid cone score
    buttonBoard.button(4).and(buttonBoard.button(16))
      .onTrue(new ExtendElevatorToPosition(elevatorGrabber, 1.21));
    buttonBoard.button(4).and(buttonBoard.button(1).or(buttonBoard.button(2)))
      .onTrue(new ScoreGamePiece(elevatorGrabber, indexer, true));

    //high cone score
    buttonBoard.button(5).and(buttonBoard.button(16))
      .onTrue(new ExtendElevatorToPosition(elevatorGrabber, 1.69));
    buttonBoard.button(5).and(buttonBoard.button(1).or(buttonBoard.button(2)))
      .onTrue(new ScoreGamePiece(elevatorGrabber, indexer, true));

    // cube is separate because possible different elevator heights and maybe reversed scoregamepiece
    //mid cube score
    buttonBoard.button(4).and(buttonBoard.button(16).negate())
      .onTrue(new ExtendElevatorToPosition(elevatorGrabber, 1.21));
    buttonBoard.button(4).and(buttonBoard.button(2))
      .onTrue(new ScoreGamePiece(elevatorGrabber, indexer, false));

    //high cube score
    buttonBoard.button(5).and(buttonBoard.button(16).negate())
      .onTrue(new ExtendElevatorToPosition(elevatorGrabber, 1.69));
    buttonBoard.button(5).and(buttonBoard.button(2))
      .onTrue(new ScoreGamePiece(elevatorGrabber, indexer, false));

    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand(drivetrain::setGyroscope180)
      .andThen(new AutoRoutine("1 cone + balance wire guard", drivetrain, vision, elevatorGrabber, indexer, intake));
  }
}
