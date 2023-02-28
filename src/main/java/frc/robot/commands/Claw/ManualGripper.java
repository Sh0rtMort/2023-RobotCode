// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Claw;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Arm;
import edu.wpi.first.wpilibj.Joystick;

public class ManualGripper extends CommandBase {
  /** Creates a new ManualGripper. */
  private final ArmSubsystem m_armSubsystem;
  private final Joystick gripperController;
  private final int gripperAxis;

  public ManualGripper(ArmSubsystem m_armSubsystem, Joystick gripperController, int gripperAxis) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_armSubsystem = m_armSubsystem;
    addRequirements(m_armSubsystem);

    this.gripperController = gripperController;
    this.gripperAxis = gripperAxis;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double gripperPosition = m_armSubsystem.getGripperPosition();

    double gripperSpeed = gripperController.getRawAxis(gripperAxis);

    if(gripperPosition <= Arm.gripperOpenPosition+3000 && gripperSpeed < 0) {
      gripperSpeed = 0;
      m_armSubsystem.stopGripper();
      System.out.println("Gripper opened all the way!!!!");
    }

    gripperSpeed = .25 * ((Math.abs(gripperSpeed) < Constants.stickDeadband) ? 0 : gripperSpeed);

    m_armSubsystem.setGripperSpeed(gripperSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Gripper Manual Command Stopped!");
    m_armSubsystem.stopGripper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
