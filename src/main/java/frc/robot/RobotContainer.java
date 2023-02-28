// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BDSM license file in the root directory of this project.  

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory; 

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Claw.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Creates new joystick object for the driver on port 0
  private final Joystick driver = new Joystick(0);
  private final Joystick arm = new Joystick(1);

  // Creates the Axis variables mapped to various joysticks on the controller
  private final int translationAxis = XboxController.Axis.kLeftY.value; //Y axis on left joystick, front to back motion
  private final int strafeAxis = XboxController.Axis.kLeftX.value; //X axis on the left joystick, left to right motion
  private final int rotationAxis = XboxController.Axis.kRightX.value; //X axis on the right joystick, turns the robot
  private final int gripperAxis = 0;

  // Creates button mappings on the controller
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value); // Y button on the controller to zero the gyro
  private final JoystickButton armUpAndOut = new JoystickButton(arm, 3); // Arm up and out
  private final JoystickButton armDownAndOut = new JoystickButton(arm, 2); // Arm down and out
  private final JoystickButton armStore = new JoystickButton(arm, 4); // Default position
  private final JoystickButton armMiddle = new JoystickButton(arm, 5); // Place object in middle row
  private final JoystickButton motorRelease = new JoystickButton(arm, 8); //Arm free fall
  private final JoystickButton zeroArmEncoders = new JoystickButton(arm, 9); 
  private final JoystickButton gripperOpen = new JoystickButton(arm, 6); //Opens claw
  private final JoystickButton gripperClose = new JoystickButton(arm, 7); //Close claw


  // Define the Swerve subsystem as swerveSubsystem
  private final Swerve swerveSubsystem = new Swerve();
  private final ArmSubsystem armSubsystem = new ArmSubsystem();

  private final ArmHigh armHigh = new ArmHigh(armSubsystem);
  private final ArmLow armLow = new ArmLow(armSubsystem);
  private final PutThoseGrippersAway armStow = new PutThoseGrippersAway(armSubsystem);
  private final ArmMid armMid = new ArmMid(armSubsystem);

  private final GripperOpen greasyGripper9000Open = new GripperOpen(armSubsystem);
  private final GripperClose greasyGripper9000Close = new GripperClose(armSubsystem);

      /* Autonomous Mode Chooser */
      private final SendableChooser<Command> autoChooser = new SendableChooser<>();



  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true; // Do you want field oriented control?
    boolean openLoop = true; 
    swerveSubsystem.setDefaultCommand(new TeleopSwerve(swerveSubsystem, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop));  //Default command to drive the bot
    //armSubsystem.setDefaultCommand(new ManualGripper(armSubsystem, arm, gripperAxis));
    // Configure the button bindings
    configureButtonBindings();

    configureSmartDashboard();
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
    
    //Arm Buttons
    armUpAndOut.whileTrue(armHigh);
    armDownAndOut.whileTrue(armLow);
    armStore.whileTrue(armStow);
    armMiddle.whileTrue(armMid);

    //Gripper Buttons
    gripperOpen.whileTrue(greasyGripper9000Open);
    gripperClose.whileTrue(greasyGripper9000Close);

    //Debug Buttons
    motorRelease.onTrue(new InstantCommand(() -> armSubsystem.releaseAllMotors()));
    motorRelease.onFalse(new InstantCommand(() -> armSubsystem.brakeAllMotors()));
    zeroArmEncoders.onTrue(new InstantCommand(() -> armSubsystem.zeroAllEncoders()));
  }

  
  private void configureSmartDashboard() {
    autoChooser.setDefaultOption("Simple Straight Out", new SequentialCommandGroup(
      new SimpleOut(swerveSubsystem)));

    autoChooser.addOption("Straight out to Dock", new SequentialCommandGroup(
      new StraightDock(swerveSubsystem)));

    SmartDashboard.putData(autoChooser);
  }

  public void disabledInit() {
    swerveSubsystem.resetModulesToAbsolute();
  }

  public void robotInit() {
    armSubsystem.zeroAllEncoders();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // the testAuto routine will run in auton
        return autoChooser.getSelected();
  }
}
