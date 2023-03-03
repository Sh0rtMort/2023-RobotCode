// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeBalance extends CommandBase {

  private final Swerve m_Swerve;
  private final PIDController xPIDController;
  private Translation2d trans;
  private Translation2d endingTranslation;

  /** Creates a new ChargeBalance. */
  public ChargeBalance(Swerve m_Swerve) {
        this.m_Swerve = m_Swerve;
        addRequirements(m_Swerve);

        this.xPIDController = new PIDController(.05, AutoConstants.kIXController, AutoConstants.kDXController);
        xPIDController.setTolerance(100);

        xPIDController.setSetpoint(0);
  }

  @Override
  public void execute() {

    double xSpeed = xPIDController.calculate(m_Swerve.getRoll());

    trans = new Translation2d(-xSpeed, 0);

    m_Swerve.drive(trans, 0, true, true);

    SmartDashboard.putNumber("Roll", m_Swerve.getRoll());
    SmartDashboard.putNumber("xSpeed", xSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    endingTranslation = new Translation2d(0, 0);
    m_Swerve.drive(endingTranslation, 0, true, true);
    System.out.println("Auto Balance Stopped!!!!!"); //Prints to the console
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
