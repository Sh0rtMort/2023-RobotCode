// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants;
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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class InnerLeaveAndLevel extends SequentialCommandGroup {
  /** Creates a new OutAndLevel. */
  public InnerLeaveAndLevel(Swerve swerveSubsystem){
      TrajectoryConfig config = new TrajectoryConfig(
                Constants.AutoConstants.kMaxSpeedMetersPerSecond, //Sets Max speed of the bot in auton
                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared) //Sets Max acceleration of the bot in auton
                    .setKinematics(Constants.Swerve.swerveKinematics); //Gets all the kinematics info for swerve

        // Basic trajectory using the traj generator tool built into WPILib
        Trajectory testingTrajectory = TrajectoryGenerator.generateTrajectory(
                // Sets the start direction
                new Pose2d(0, 0, new Rotation2d(0)),
                // Should go in a straight line
                List.of(
                    new Translation2d(1, 0), //1st point 1 meter ahead of where we started
                    new Translation2d(2, 0), //2nd point 2 meters ahead of where we started
                    new Translation2d(3, 0), //2nd point 2 meters ahead of where we started
                    new Translation2d(4, 0), //1st point 1 meter ahead of where we started
                    new Translation2d(5, 0), //2nd point 2 meters ahead of where we started
                    new Translation2d(5, -1), //2nd point 2 meters ahead of where we started
                    new Translation2d(5, -2), //1st point 1 meter ahead of where we started
                    new Translation2d(4, -2), //2nd point 2 meters ahead of where we started
                    new Translation2d(3, -2)), //2nd point 2 meters ahead of where we started
                new Pose2d(3, -2, new Rotation2d(0)), //Sets end pose 3 meters ahead of starting point
                config);

        PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                testingTrajectory,
                swerveSubsystem::getPose,
                Constants.Swerve.swerveKinematics,
                xController,
                yController,
                thetaController,
                swerveSubsystem::setModuleStates,
                swerveSubsystem);


        addCommands(
            new InstantCommand(() -> swerveSubsystem.resetOdometry(testingTrajectory.getInitialPose())),
            swerveControllerCommand);
    }
}