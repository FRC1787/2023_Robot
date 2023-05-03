// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intakeIndex.Intake;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.commands.elevatorGrabber.ExtendElevatorToPosition;
import frc.robot.commands.elevatorGrabber.MoveElevatorToPosition;
import frc.robot.commands.elevatorGrabber.PickUpCone;
import frc.robot.commands.elevatorGrabber.PickUpCube;
import frc.robot.commands.elevatorGrabber.ScoreGamePiece;
import frc.robot.commands.elevatorGrabber.SetGrabberMotor;
import frc.robot.commands.indexer.IndexConeFull;
import frc.robot.commands.indexer.MoveSideBelts;
import frc.robot.commands.intake.BowlCube;
import frc.robot.commands.intake.IntakeGamePieces;
import frc.robot.commands.intake.MoveConveyor;
import frc.robot.commands.intake.MoveIntakeWheels;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.GrabberPlacer;
import frc.robot.subsystems.elevator.Pivot;
import frc.robot.subsystems.intakeIndex.Claw;
import frc.robot.subsystems.intakeIndex.IndexerWalls;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutine extends SequentialCommandGroup {
  /** Creates a new AutoRoutine. */
  public AutoRoutine(
      List<PathPlannerTrajectory> pathGroup, 
      Drivetrain drivetrain, 
      Vision vision, 
      GrabberPlacer grabberPlacer, 
      Elevator elevator, 
      Pivot pivot, 
      IndexerWalls indexerWalls, 
      Claw claw,
      Intake intake,
      Conveyor conveyor
    ) {

    HashMap<String, Command> eventMap = new HashMap<>();



  // INTAKING //////////////////////////////////
    // eventMap.put("intakeOut", new IntakeGamePieces(intake, conveyor, indexerWalls, pivot, -3, -5, -6));
    eventMap.put("intakeOut", new SequentialCommandGroup(
      new InstantCommand(intake::extendIntake, intake),
      new InstantCommand(indexerWalls::openIndexerWalls, indexerWalls),
      new ParallelCommandGroup(
        new MoveIntakeWheels(intake, -3),
        new MoveConveyor(conveyor, -5),
        new MoveSideBelts(indexerWalls, -6)
      )
    ));
    //this command will interrupt intakeOut, which will in turn run the end()
    //method of IntakeGamePieces, which retracts the intake
    eventMap.put("intakeIn",
      new SequentialCommandGroup(
        new InstantCommand(intake::retractIntake),
        new ParallelCommandGroup(
          new MoveIntakeWheels(intake, -3),
          new MoveConveyor(conveyor, -5),
          new MoveSideBelts(indexerWalls, -6)
        ).withTimeout(0.7)
      )
    );

  // INDEXING //////////////////////////////////////
    eventMap.put("indexCone", new IndexConeFull(intake, conveyor, indexerWalls, claw, elevator, pivot));
    eventMap.put("indexCube", new PickUpCube(intake, conveyor, elevator, pivot, grabberPlacer, indexerWalls));
    eventMap.put("pickUpCone", new PickUpCone(elevator, pivot, grabberPlacer, intake, conveyor, indexerWalls, claw));


  // HIGH/MID SCORING ////////////////////////
    eventMap.put("scoreCubeHigh",
      // This can inturrupt index cube, so we want the indexer walls to be open when we score.
      new InstantCommand(indexerWalls::openIndexerWalls).andThen(
        new ParallelRaceGroup(
          new SetGrabberMotor(grabberPlacer, -0.75, 100), // apply small holding torque to keep cube in grabber on the way up
          new ExtendElevatorToPosition(elevator, pivot, 1.6).withTimeout(1.3) //if elevator doesn't reach setpoint in time, score game piece anyways
        ).andThen(
          new ScoreGamePiece(elevator, pivot, grabberPlacer, false)
        )
      )
    );

    
    eventMap.put("placeCubeHigh", 
    new SequentialCommandGroup(
      new ExtendElevatorToPosition(elevator, pivot, 1.6),
      new SetGrabberMotor(grabberPlacer, 6, 100).withTimeout(.15)
      )
    );

    eventMap.put("scoreConeHigh", 
      new SequentialCommandGroup(
        new SetGrabberMotor(grabberPlacer, 6, 24).withTimeout(2.0), // initial cone suck into back stop
        new ExtendElevatorToPosition(elevator, pivot, 1.69),
        new ScoreGamePiece(elevator, pivot, grabberPlacer, true))
    );

    eventMap.put("placeConeHigh", 
    new SequentialCommandGroup(
      new SetGrabberMotor(grabberPlacer, 6, 24).withTimeout(2.0), // initial cone suck into back stop
      new ExtendElevatorToPosition(elevator, pivot, 1.69),
      new SetGrabberMotor(grabberPlacer, -6, 100).withTimeout(0.15), // placement onto peg
      new SetGrabberMotor(grabberPlacer, 6, 100).withTimeout(0.15) // move cube hat out of the way
      )
    );

    eventMap.put("retractElevator",
      new ParallelCommandGroup(
        // reset the elevator and indexer walls to prepare for getting the next game piece
        new MoveElevatorToPosition(elevator, 0).asProxy(),
        new InstantCommand(pivot::retractElevator, pivot),
        new InstantCommand(indexerWalls::openIndexerWalls, indexerWalls)
      )
    );

    //spit out cube from intake in case cube doesnt get in grabber
    eventMap.put("backUpScore",
      new InstantCommand(indexerWalls::closeIndexerWalls).andThen(
        new ParallelCommandGroup(
          new MoveConveyor(conveyor, 2.5),
          new MoveSideBelts(indexerWalls, 2.5),
          new MoveIntakeWheels(intake, 6.75)
        ).withTimeout(3)
      )
    );


  // LOW SCORING ////////////////////////////////////////////
    eventMap.put("shootCube", new SequentialCommandGroup(
      new BowlCube(intake, conveyor, indexerWalls, grabberPlacer, 6.75, 2.5, 2.5, 0).withTimeout(3)
    ));

    eventMap.put("shootCubeChargeStation", new SequentialCommandGroup(
      new WaitCommand(1.25),
      new BowlCube(intake, conveyor, indexerWalls, grabberPlacer, 6.75, 2.5, 2.5, 0).withTimeout(3)
    ));


  // BALANCE ////////////////
    eventMap.put("waitOneSecond", new WaitCommand(1));
    
    eventMap.put("autoBalance", new AutoBalance(drivetrain));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPoseMeters,
      drivetrain::setPoseMeters,
      Constants.Swerve.swerveKinematics,
      new PIDConstants(16.0, 0, 0),
      new PIDConstants(0.25, 0, 0),
      //new PIDConstants(0, 0, 0),
      //new PIDConstants(0, 0, 0),
      drivetrain::setModuleStatesClosedLoop,
      eventMap,
      true,
      drivetrain
    );
    
    addCommands(
      autoBuilder.fullAuto(pathGroup)
    );
  }
}
