package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class executeTrajectory extends SequentialCommandGroup {

  public final PathPlannerTrajectory trajectory;

  public executeTrajectory(Swerve s_Swerve, PathPlannerTrajectory trajectory, boolean setInitialPose) {

    PathPlannerTrajectory moveOut = PathPlanner.loadPath("Simple Out.path", Constants.AutoConstants.kMaxSpeedMetersPerSecond, Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared);
    this.trajectory = trajectory;

    s_Swerve.getField().getObject("Field").setTrajectory(trajectory);

    PIDController thetaController = new PIDController(
        Constants.AutoConstants.kPThetaController,
        0,
        0);

    PPSwerveControllerCommand swerveControllerCommand = new PPSwerveControllerCommand(
        moveOut,
        s_Swerve::getPose,
        Constants.Swerve.swerveKinematics,
        new PIDController(Constants.AutoConstants.kPXController, 0, 0),
        new PIDController(Constants.AutoConstants.kPYController, 0, 0),
        thetaController,
        s_Swerve::setModuleStates,
        s_Swerve);

        if (setInitialPose) {
          addCommands(
              new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialHolonomicPose())),
              swerveControllerCommand);
        } else {
          addCommands(
            new InstantCommand(() -> s_Swerve.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand);
  }
}
}