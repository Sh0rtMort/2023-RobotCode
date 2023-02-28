// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Arm.ArmMid;
import frc.robot.commands.Claw.GripperOpen;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DropCubeMid extends SequentialCommandGroup {
  public DropCubeMid(Swerve swerveSubsystem, ArmSubsystem armSubsystem) {

    TrajectoryConfig config = new TrajectoryConfig(
      Constants.AutoConstants.kMaxSpeedMetersPerSecond, //Sets Max speed of the bot in auton
      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared) //Sets Max acceleration of the bot in auton
          .setKinematics(Constants.Swerve.swerveKinematics); //Gets all the kinematics info for swerve

    Trajectory driveToGridTraj = TrajectoryGenerator.generateTrajectory(
          // Sets the start direction
          new Pose2d(1, 0, new Rotation2d(180)),
          List.of(
              new Translation2d(.75, 0), //First internal waypoint
              new Translation2d(.5, 0)), //Second internal waypoint
          new Pose2d(0, 0, new Rotation2d(180)), //Get to charge station
          config);

    Trajectory leaveCommunityZoneTraj = TrajectoryGenerator.generateTrajectory(
      // Sets the start direction
      new Pose2d(0, 0, new Rotation2d(180)),
      // Should go in a straight line
      List.of(
          new Translation2d(1, 0), //First internal waypoint
          new Translation2d(2, 0), //Second internal waypoint
          new Translation2d(3, 0)),//Third internal waypoint
      new Pose2d(4, 0, new Rotation2d(0)), //Get to resting position to start match
      config);

    PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand firstSwerveControllerCommand = new SwerveControllerCommand(
      driveToGridTraj,
      swerveSubsystem::getPose,
      Constants.Swerve.swerveKinematics,
      xController,
      yController,
      thetaController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem);

    SwerveControllerCommand secondSwerveControllerCommand = new SwerveControllerCommand(
      leaveCommunityZoneTraj,
      swerveSubsystem::getPose,
      Constants.Swerve.swerveKinematics,
      xController,
      yController,
      thetaController,
      swerveSubsystem::setModuleStates,
      swerveSubsystem);

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelCommandGroup(
      new ArmMid(armSubsystem),
      new InstantCommand(() -> swerveSubsystem.resetOdometry(driveToGridTraj.getInitialPose())),
      firstSwerveControllerCommand
    )
    );
    addCommands(new SequentialCommandGroup(
      new WaitCommand(1),
      new GripperOpen(armSubsystem),
      new WaitCommand(0.5),
      new InstantCommand(() -> swerveSubsystem.resetOdometry(leaveCommunityZoneTraj.getInitialPose())),
      secondSwerveControllerCommand
    )

    );
  }
}
