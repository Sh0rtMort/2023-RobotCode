// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.Swerve.Arm;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.*;

public class GripperOpen extends CommandBase {

  private final ArmSubsystem m_armSubsystem;
  private final PIDController gripperPIDController;
  
  public GripperOpen(ArmSubsystem m_armSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
      this.m_armSubsystem = m_armSubsystem;
      addRequirements(m_armSubsystem);

      this.gripperPIDController = new PIDController(Arm.gripperKP, Arm.gripperKI, Arm.gripperKD);
      gripperPIDController.setTolerance(1);
      gripperPIDController.setSetpoint(Arm.gripperOpenPosition);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Gripper opening!!!!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double gripperSpeed = gripperPIDController.calculate(m_armSubsystem.getGripperPosition());

    m_armSubsystem.setGripperSpeed(gripperSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Stopping the gripper!!!");
    m_armSubsystem.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
