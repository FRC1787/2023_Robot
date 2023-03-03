// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.autonomous.AutoRoutine;
import frc.robot.commands.drivetrain.JoystickDrive;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPosition;
import frc.robot.commands.elevatorGrabber.SetGrabberMotor;
import frc.robot.commands.intakeIndex.IntakeGamePieces;
import frc.robot.commands.intakeIndex.MoveClawBack;
import frc.robot.commands.intakeIndex.MoveClawForward;
import frc.robot.commands.intakeIndex.MoveConveyor;
import frc.robot.commands.intakeIndex.MoveSideBelts;
import frc.robot.commands.intakeIndex.UprightCone;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ElevatorGrabber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();
  private final Vision vision = new Vision();
  private final ElevatorGrabber elevatorGrabber = new ElevatorGrabber();
  private final LED led = new LED();

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

    drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, false));

    controller.a().whileTrue(new MoveElevatorToPosition(elevatorGrabber, 1.5));
    controller.b().whileTrue(new MoveElevatorToPosition(elevatorGrabber, 0));

    controller.povDown().whileTrue(new MoveSideBelts(indexer, -0.3));
    controller.povUp().whileTrue(new MoveSideBelts(indexer, 0.3));
    controller.povRight().whileTrue(new UprightCone(indexer, intake));

    controller.y().whileTrue(new IntakeGamePieces(intake, indexer, -3, -12));

    controller.leftBumper().whileTrue(new MoveClawForward(indexer, 0.2));
    controller.rightBumper().whileTrue(new MoveClawBack(indexer, 0.2));
    controller.leftTrigger().whileTrue(new MoveConveyor(intake, 0.25));
    controller.rightTrigger().whileTrue(new MoveConveyor(intake, -0.25));
    
    intakeIn.onTrue(new InstantCommand(intake::retractIntake));
    intakeOut.onTrue(new InstantCommand(intake::extendIntake));

    extendElevator.onTrue(new InstantCommand(elevatorGrabber::extendElevator));
    retractElevator.onTrue(new InstantCommand(elevatorGrabber::retractElevator));

    sideBeltsIn.onTrue(new InstantCommand(indexer::closeIndexerWalls));
    sideBeltsOut.onTrue(new InstantCommand(indexer::openIndexerWalls));

    //buttonBoard.button(0).onTrue(new InstantCommand(led::setYellow).andThen(new InstantCommand(indexer::setConeMode)));
    //buttonBoard.button(0).onFalse(new InstantCommand(led::setYellow).andThen(new InstantCommand(indexer::setCubeMode)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new AutoRoutine(autoChooser.getSelected(), drivetrain, vision);
  }
}
