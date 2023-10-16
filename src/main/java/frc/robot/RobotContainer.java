// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BDSM license file in the root directory of this project.  

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.IntakeControl;
import frc.robot.commands.ArmCommand;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Joystick driver = new Joystick(0);
  // private final Joystick arm = new Joystick(1);

  // Creates the Axis variables mapped to various joysticks on the gamepad
  private final int translationAxis = XboxController.Axis.kLeftY.value; //Y axis on left joystick, front to back motion
  private final int strafeAxis = XboxController.Axis.kLeftX.value; //X axis on the left joystick, left to right motion
  private final int rotationAxis = XboxController.Axis.kRightX.value; //X axis on the right joystick, turns the robot

  // Creates button mappings on the controller
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); // Y button on the controller to zero the gyro
  private final JoystickButton intakeSuckButton = new JoystickButton(driver, 5);
  private final JoystickButton intakeSpitButton = new JoystickButton(driver, 6);
  private final JoystickButton armUpButton = new JoystickButton(driver, 3);
  private final JoystickButton armDownButton = new JoystickButton(driver, 2);

  // private final JoystickButton motorRelease = new JoystickButton(driver, XboxController.Button.kStart);

  // Define the Swerve subsystem as swerveSubsystem
  private final Swerve swerveSubsystem = new Swerve();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

    private final IntakeControl intakeSuck = new IntakeControl(armSubsystem, 0.55);
    private final IntakeControl intakeSpit = new IntakeControl(armSubsystem, -0.85);

    private final ArmCommand armUp = new ArmCommand(armSubsystem, -0.8);
    private final ArmCommand armDown = new ArmCommand(armSubsystem, 0.8);



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = false; // Do you want field oriented control?
    boolean openLoop = true; 
    swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));  //Default command to drive the bot
    // Configure the button bindings
    configureButtonBindings();

    armSubsystem.zeroAllEncoders();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    
    //Drive Buttons
    zeroGyro.onTrue(new InstantCommand(() -> swerveSubsystem.zeroGyro()));

    armUpButton.whileTrue(armUp);
    armDownButton.whileTrue(armDown);

    intakeSpitButton.whileTrue(intakeSpit);
    intakeSuckButton.whileTrue(intakeSuck);


    //Debug Buttons
    // motorRelease.onTrue(new InstantCommand(() -> armSubsystem.releaseAllMotors()));
    // motorRelease.onFalse(new InstantCommand(() -> armSubsystem.brakeAllMotors()));
    // zeroArmEncoders.onTrue(new InstantCommand(() -> armSubsystem.zeroAllEncoders()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // the testAuto routine will run in auton
    return new MidCharge(swerveSubsystem);
  }
}
