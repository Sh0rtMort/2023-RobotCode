// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.*;
import frc.robot.Constants.Swerve.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class PutThoseGrippersAway extends CommandBase {

  private final ArmSubsystem m_armSubsystem;
  private final PIDController shoulderPIDController;
  private final PIDController elbowPIDController;
  private final PIDController wristPIDController;

    /** Creates a new PutThoseGrippersAway. */
    public PutThoseGrippersAway(ArmSubsystem m_armSubsystem) {
          this.m_armSubsystem = m_armSubsystem;
          addRequirements(m_armSubsystem);

          this.shoulderPIDController = new PIDController(Arm.shoulderKP, Arm.shoulderKI, Arm.shoulderKD); //Input the PID values for the shoulder
          shoulderPIDController.setTolerance(.1); //Sets the tolerance for the sholder PID controller
          shoulderPIDController.setSetpoint(Arm.shoulderStorePosition); //Sets the setpoint for the shoulder PID controller

          this.elbowPIDController = new PIDController(Arm.elbowKP, Arm.elbowKI, Arm.elbowKD); //Input the PID values for the elbow
          elbowPIDController.setTolerance(.1); //Sets the tolerance for the elbow PID controller
          elbowPIDController.setSetpoint(Arm.elbowStorePosition); //Sets the setpoint for the elbow PID controller

          this.wristPIDController = new PIDController(Arm.wristKP, Arm.wristKI, Arm.wristKD); //Input the PID values for the wrist
          wristPIDController.setTolerance(.1); //Sets the tolerance for the wrist PID controller
          wristPIDController.setSetpoint(Arm.wristStorePosition); //Sets the setpoint for the wrist PID controller
    }
    

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("PutThoseGrippersAway(Arm Storage) command started. Everyone stand back, I know what I'm doing. Trust me."); //Prints to the console that the command has started
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shoulderSpeed = shoulderPIDController.calculate(m_armSubsystem.getShoulderAngle()); //Calculates the error for the shoulder
    double elbowSpeed = elbowPIDController.calculate(m_armSubsystem.getElbowAngle()); //Calculates the error for the elbow
    double wristSpeed = wristPIDController.calculate(m_armSubsystem.getWristAngle()); //Calculates the error for the wrist

    //Disable shoulder motor cuz its always broken lel
    m_armSubsystem.setSpeeds(0, elbowSpeed, wristSpeed); //Sets the speeds for the arm motors

    System.out.println("Shoulder Angle: " + m_armSubsystem.getShoulderAngle()); //Prints the current angle of the shoulder to the console
    System.out.println("Elbow Angle: " + m_armSubsystem.getElbowAngle()); //Prints the current angle of the elbow to the console
    System.out.println("Wrist Angle: " + m_armSubsystem.getWristAngle());  //Prints the current angle of the wrist to the console

    SmartDashboard.putNumber("Shoulder Angle: ", m_armSubsystem.getShoulderAngle()); //Puts the current angle of the shoulder on the SmartDashboard
    SmartDashboard.putNumber("Elbow Angle: ", m_armSubsystem.getElbowAngle()); //Puts the current angle of the elbow on the SmartDashboard
    SmartDashboard.putNumber("Wrist Angle: ", m_armSubsystem.getWristAngle()); //Puts the current angle of the wrist on the SmartDashboard
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_armSubsystem.stopMotor(); //Stops the arm motors
    System.out.println("Arm Motors Stopped!!!!!"); //Prints to the console that the arm motors have stopped
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

