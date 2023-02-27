package frc.robot.autos;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import java.util.HashMap;
import java.util.List;

import frc.robot.commands.Arm.*;
import frc.robot.commands.Claw.*;


public class executeAuto extends SequentialCommandGroup {
  public executeAuto(Swerve swerveSubsystem, PathPlannerTrajectory trajectory) {
    swerveSubsystem.getField().getObject("Field").setTrajectory(trajectory);

/*     HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("intakeDown", new ArmMid(armSubsystem)); */

    PIDController thetaController = new PIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        trajectory,
        swerveSubsystem::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        swerveSubsystem::setModuleStates,
        swerveSubsystem);

    addCommands(
        new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialHolonomicPose())),
        swerveControllerCommand);
  }

}