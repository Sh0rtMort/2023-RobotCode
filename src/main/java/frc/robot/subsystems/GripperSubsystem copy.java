// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax.IdleMode;

/** Add your docs here. */
public class GripperSubsystem extends SubsystemBase {

 private TalonFX gripperMotor = new TalonFX(21);

 private double turningSpeed = 0;

//Set neuteral mode to brake for each motor
public final void init() {
  shoulderFalcon.setNeutralMode(NeutralMode.Brake);
  clawMotor.setIdleMode(IdleMode.kBrake);
  zeroAllEncoders();

  //Set open and closed loop ramp rates
  gripperMotor.setIdleMode(IdleMode.kBrake);
  gripperMotor.setOpenLoopRampRate(0);
  gripperMotor.setClosedLoopRampRate(0);

  //Set the encoder values to read every 30 ms
  gripperMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 40);

}

  public void setSpeeds(double turningSpeed) {
    this.turningSpeed = turningSpeed;   
    gripperMotor.set(turningSpeed);

  }

public void stopMotor() {
  gripperMotor.set(0);
}


public double getElbowAngle() {
  return gripperMotor.getEncoder().getPosition();
}

public double getElbowDegrees() {
  return gripperMotor.getEncoder().getPosition() * 90 / 360; //TODO Compensate for gearbox ratio
}

public void zeroAllEncoders() {
  gripperMotor.getEncoder().setPosition(0);
  System.out.println("Gripper Encoders Zeroed");
}

public void releaseGripperMotor() {
gripperMotor.setIdleMode(IdleMode.kCoast);
System.out.println("Motors Released!");
}

public void brakeAllMotors() {
  gripperMotor.setIdleMode(IdleMode.kBrake);
  System.out.println("Motors Braked!");
  }

@Override
public void periodic(){
  SmartDashboard.putNumber("Gripper Position: ", getGripperAngle());
  SmartDashboard.putNumber("Gripper Speed: ", turningSpeed);
}

}