// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;

public class AlignToTarget extends CommandBase {

  Drivetrain drivetrain;
  Vision vision;
  Constants.Vision.LimelightTarget target;
  PIDController lateralPID = new PIDController(3, 0, 0);
  PIDController distancePID = new PIDController(-3, 0, 0);

  public AlignToTarget(Drivetrain drivetrain, Vision vision, Constants.Vision.LimelightTarget target) {
    addRequirements(drivetrain);
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.target = target;

    lateralPID.setTolerance(0.01);
    distancePID.setTolerance(0.01);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    lateralPID.setSetpoint(Constants.Vision.limelightLateralOffsetMeters);
    distancePID.setSetpoint(target.distanceMeters + Constants.Vision.limelightBumperDistanceMeters);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double lateralOffsetMeters = vision.getLateralOffsetMeters(target);
    double distanceOffsetMeters = vision.getTargetDistanceMeters(target);

    double lateralVel = MathUtil.clamp(
      lateralPID.calculate(lateralOffsetMeters),
      -1,
      1
    );

    double distanceVel;
    if (distanceOffsetMeters == 0.0)
      distanceVel = 0;
    else {
      distanceVel = MathUtil.clamp(
        distancePID.calculate(distanceOffsetMeters),
        -1,
        1
      );
    }
    
    drivetrain.drive(
      new ChassisSpeeds(distanceVel, lateralVel, 0)
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(
      new ChassisSpeeds(0, 0, 0)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lateralPID.atSetpoint() && distancePID.atSetpoint();
  }
}
