// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intakeIndex.Intake;
import frc.robot.subsystems.intakeIndex.Conveyor;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.commands.elevatorGrabber.ExtendElevatorToPosition;
import frc.robot.commands.elevatorGrabber.PickUpCone;
import frc.robot.commands.elevatorGrabber.PickUpCube;
import frc.robot.commands.elevatorGrabber.ScoreGamePiece;
import frc.robot.commands.elevatorGrabber.SetGrabberMotor;
import frc.robot.commands.indexer.IndexConeFull;
import frc.robot.commands.intake.BowlCube;
import frc.robot.commands.intake.IntakeGamePieces;
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
      String path, 
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
    
    double maxVelocityMetersPerSecond = 1.0; // 4.0;
    double accelerationMetersPerSecondSquared = 0.5; //2.5;
    if (path.equals("1 cone + balance middle") || path.equals("1 cone middle")) {
      maxVelocityMetersPerSecond = 4.0;
      accelerationMetersPerSecondSquared = 2.0;
    }

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
      path, new PathConstraints(maxVelocityMetersPerSecond, accelerationMetersPerSecondSquared)); //3, 1.5 is previous but it is not fast enough with square paths

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("align", new AlignToTarget(drivetrain, vision, Constants.Vision.LimelightTarget.midTape));
    eventMap.put("autoBalance", new AutoBalance(drivetrain));
    eventMap.put("pickUpCone", new PickUpCone(elevator, pivot, grabberPlacer, intake, conveyor, indexerWalls, claw));
    eventMap.put("scoreConeHigh", 
      new SequentialCommandGroup(
        new SetGrabberMotor(grabberPlacer, 6, 25).withTimeout(0.5),
        new ExtendElevatorToPosition(elevator, pivot, 1.69),
        new ScoreGamePiece(elevator, pivot, grabberPlacer, indexerWalls, true))
    );
    eventMap.put("intakeOut", new IntakeGamePieces(intake, conveyor, indexerWalls, pivot, -3, -5, -6));
    eventMap.put("indexCube", new PickUpCube(intake, conveyor, elevator, pivot, grabberPlacer, indexerWalls));
    eventMap.put("intakeIn", new InstantCommand(intake::stopIntakeMotor).andThen(new InstantCommand(intake::retractIntake)));
    eventMap.put("indexCone", new IndexConeFull(intake, conveyor, indexerWalls, claw, elevator, pivot));
    eventMap.put("shootCube", new SequentialCommandGroup(
      new WaitCommand(2),
      new BowlCube(intake, conveyor, indexerWalls, grabberPlacer, 7, 3, 3, 0).withTimeout(3)
    ));
    eventMap.put("waitOneSecond", new WaitCommand(1));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPoseMeters,
      drivetrain::setPoseMeters,
      Constants.Swerve.swerveKinematics,
      new PIDConstants(0, 0, 0), // 15: TODO: TUNE THESE AFTER ODOMETRY GHOST BUSTED
      new PIDConstants(0.5, 0, 0), //5 RADIANS DOESN'T MAKE SENSE MAYHAPS???
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
