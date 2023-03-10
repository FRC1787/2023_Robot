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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.ElevatorGrabber;
import frc.robot.commands.drivetrain.AlignToTarget;
import frc.robot.commands.elevatorGrabber.ScoreGamePiece;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoRoutine extends SequentialCommandGroup {
  /** Creates a new AutoRoutine. */
  public AutoRoutine(String path, Drivetrain drivetrain, Vision vision, frc.robot.subsystems.ElevatorGrabber elevatorGrabber) {
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
      path, new PathConstraints(3, 1.5));

    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("align", new AlignToTarget(drivetrain, vision, Constants.Vision.LimelightTarget.midTape));
    eventMap.put("autoBalance", new AutoBalance(drivetrain));
    eventMap.put("scoreHigh", new ScoreGamePiece(elevatorGrabber, 1.69));

    // TODO: tune these pid constants to be the same as the AlignToTarget (inputs to pid are desired position error (m) and output is velocity (m/s))
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      drivetrain::getPoseMeters,
      drivetrain::setPoseMeters,
      Constants.Swerve.swerveKinematics,
      new PIDConstants(20, 0, 0),
      new PIDConstants(5, 0, 0),
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
