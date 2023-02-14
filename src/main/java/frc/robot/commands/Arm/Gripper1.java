// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Gripper extends CommandBase{
  private final GripperSubsystem gripperSubsystem;
  private final Joystick controller;
  private final PIDController shoulderPIDController;
  private final PIDController elbowPIDController;
  private final PIDController wristPIDController;
  
  public Gripper
(GripperSubsystem m_gripperSubsystem) {
        this.m_gripperSubsystem = m_gripperSubsystem;
        addRequirements(m_GripperSubsystem);

        this.gripperPIDController = new PIDController(Gripper.gripperKP, Gripper.gripperKI, Gripper.gripperKP);
        gripperPIDController.setTolerance(.1);
        gripperPIDController.setSetpoint(Gripper.gripperSliderPosition);
  }
  
  // Called when the command is initially scheduled.
@Override
public void initialize() {
  System.out.println("Gripper command started.");
}

// Called every time the scheduler runs while the command is scheduled.
@Override
public void execute() {
  double TurningSpeed = gripperPIDController.calculate(m_gripperSubsystem.getGripperAngle());

  m_gripperSubsystem.setSpeeds(turningSpeed);

  System.out.println("Gripper Angle: " + m_gripperSubsystem.getGripperAngle());

  SmartDashboard.putNumber("Gripper Angle: ", m_gripperSubsystem.getGripperAngle());
}

// Called once the command ends or is interrupted.
@Override
public void end(boolean interrupted) {
  m_gripperSubsystem.stopMotor();
  System.out.println("Gripper Motors Stopped!!!!!");
}

// Returns true when the command should end.
@Override
public boolean isFinished() {
  return false;
}
}