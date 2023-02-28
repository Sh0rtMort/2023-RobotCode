// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.ArmSubsystem;



public class TeleopArm extends CommandBase {
 

  private ArmSubsystem armSubsystem;
  private boolean openLoop;
  private Joystick controller;
  private int gripperAxis;
  private int armAxis;

  //Control Stuff for the arm
  public TeleopArm(ArmSubsystem armSubsystem, Joystick controller, int armAxis, int gripperAxis, boolean openLoop) {
    this.armSubsystem = armSubsystem;
        addRequirements(armSubsystem);

        //Makes the variables defined earlier equal the values from RobotContainer
        this.controller = controller;
        this.armAxis = armAxis;
        this.gripperAxis = gripperAxis;
        this.openLoop = openLoop;
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double kX = controller.getRawAxis(gripperAxis); 
    double kY = -controller.getRawAxis(armAxis);
  }
}