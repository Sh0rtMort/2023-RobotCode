// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ArmHigh extends CommandBase  {

  private final ArmSubsystem m_armSubsystem;
  private final PIDController shoulderPIDController;
  private final PIDController elbowPIDController;
  private final PIDController wristPIDController;
/*   private final double shoulderHighAngle;
  private final double elbowHighAngle;
  private final double wristHighAngle; */

    /** Creates a new ArmHigh. */
    public ArmHigh(ArmSubsystem m_armSubsystem) {
          this.m_armSubsystem = m_armSubsystem;
          addRequirements(m_armSubsystem);

          this.shoulderPIDController = new PIDController(Arm.shoulderKP, Arm.shoulderKI, Arm.shoulderKP);
          shoulderPIDController.setTolerance(2);
          shoulderPIDController.setSetpoint(Arm.shoulderHighPosition);

          this.elbowPIDController = new PIDController(Arm.elbowKP, Arm.elbowKI, Arm.elbowKP);
          elbowPIDController.setTolerance(2);
          elbowPIDController.setSetpoint(Arm.elbowHighPosition);

          this.wristPIDController = new PIDController(Arm.wristKP, Arm.wristKI, Arm.wristKP);
          wristPIDController.setTolerance(.5);
          wristPIDController.setSetpoint(Arm.wristHighPosition);
    }
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ArmHigh command started. Everone stand back, I know what I'm doing. Trust me.");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSpeed = shoulderPIDController.calculate(m_armSubsystem.getShoulderAngle());
    double elbowSpeed = elbowPIDController.calculate(m_armSubsystem.getElbowAngle());
    double wristSpeed = 1.5*wristPIDController.calculate(m_armSubsystem.getWristAngle());

    m_armSubsystem.setSpeeds(0, elbowSpeed, wristSpeed);

    System.out.println("Shoulder Angle: " + m_armSubsystem.getShoulderAngle());
    System.out.println("Elbow Angle: " + m_armSubsystem.getElbowAngle());
    System.out.println("Wrist Angle: " + m_armSubsystem.getWristAngle());

    SmartDashboard.putNumber("Shoulder Angle: ", m_armSubsystem.getShoulderAngle());
    SmartDashboard.putNumber("Elbow Angle: ", m_armSubsystem.getElbowAngle());
    SmartDashboard.putNumber("Wrist Angle: ", m_armSubsystem.getWristAngle());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopMotor();
    System.out.println("Arm Motors Stopped!!!!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

