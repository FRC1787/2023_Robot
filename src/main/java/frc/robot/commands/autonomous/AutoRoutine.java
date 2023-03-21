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
import frc.robot.subsystems.ElevatorGrabber;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.commands.elevatorGrabber.ExtendElevatorToPosition;
import frc.robot.commands.elevatorGrabber.PickUpCone;
import frc.robot.commands.elevatorGrabber.PickUpCube;
import frc.robot.commands.elevatorGrabber.ScoreGamePiece;
import frc.robot.commands.elevatorGrabber.SetGrabberMotor;
import frc.robot.commands.indexer.IndexConeFull;
import frc.robot.commands.intake.EjectGamePiece;
import frc.robot.commands.intake.IntakeGamePieces;
import frc.robot.subsystems.CubeHatHack;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutine extends SequentialCommandGroup {
  /** Creates a new AutoRoutine. */
  public AutoRoutine(String path, Drivetrain drivetrain, Vision vision, ElevatorGrabber elevatorGrabber, Indexer indexer, Intake intake, CubeHatHack hatHack) {
    double maxVelocityMetersPerSecond = 4.0;
    double accelerationMetersPerSecondSquared = 2.5;
    if (path.equals("1 cone + balance middle") || path.equals("1 cone middle")) {
      maxVelocityMetersPerSecond = 4.0;
      accelerationMetersPerSecondSquared = 2.0;
    }

    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
      path, new PathConstraints(maxVelocityMetersPerSecond, accelerationMetersPerSecondSquared)); //3, 1.5 is previous but it is not fast enough with square paths

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("align", new AlignToTarget(drivetrain, vision, Constants.Vision.LimelightTarget.midTape));
    eventMap.put("autoBalance", new AutoBalance(drivetrain));
    eventMap.put("pickUpCone", new PickUpCone(elevatorGrabber, intake, indexer));
    eventMap.put("scoreConeHigh", 
      new SequentialCommandGroup(
        new SetGrabberMotor(elevatorGrabber, 6, 25).withTimeout(0.5),
        new ExtendElevatorToPosition(elevatorGrabber, 1.69),
        new ScoreGamePiece(elevatorGrabber, indexer, true))
    );
    eventMap.put("intakeOut", new IntakeGamePieces(intake, indexer, elevatorGrabber, -4, -12, -6));
    eventMap.put("indexCube", new PickUpCube(intake, elevatorGrabber, indexer, hatHack));
    eventMap.put("intakeIn", new InstantCommand(intake::stopIntakeMotors).andThen(new InstantCommand(intake::retractIntake)));
    eventMap.put("indexCone", new IndexConeFull(intake, indexer, elevatorGrabber));
    eventMap.put("shootCube", new SequentialCommandGroup(
      new WaitCommand(0.75),
      new EjectGamePiece(intake, indexer, elevatorGrabber, 12, 8, 8, 6).withTimeout(1)
    ));
    eventMap.put("waitOneSecond", new WaitCommand(1));

    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPoseMeters,
      drivetrain::setPoseMeters,
      Constants.Swerve.swerveKinematics,
      new PIDConstants(0, 0, 0), // 15: TODO: TUNE THESE AFTER ODOMETRY GHOST BUSTED
      new PIDConstants(0, 0, 0), //5 RADIANS DOESN'T MAKE SENSE MAYHAPS???
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
