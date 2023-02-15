// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Swerve;
import org.photonvision.PhotonCamera;

public class AlignBottom extends CommandBase {

  private final Swerve m_swerveSubsystem;
  private final Translation2d trans2D;
  private final PIDController strafePIDController;
  private final PIDController frontBackPIDController;
  private final PhotonCamera camera;

  public AlignBottom(Swerve m_swerveSubsystem) {
    this.m_swerveSubsystem = m_swerveSubsystem;
    addRequirements(m_swerveSubsystem);

    this.camera = new PhotonCamera(Constants.Targeting.frontCamera);

    this.strafePIDController = new PIDController(Constants.Targeting.strafeKP, Constants.Targeting.strafeKI, Constants.Targeting.strafeKD);
    strafePIDController.setTolerance(1);
    strafePIDController.setSetpoint(0);

    this.frontBackPIDController = new PIDController(Constants.Targeting.translationKP, Constants.Targeting.translationKI, Constants.Targeting.translationKD);
    frontBackPIDController.setTolerance(1);
    frontBackPIDController.setSetpoint(Constants.Targeting.TargetDistance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
