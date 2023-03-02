package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class TeleopSwerve extends CommandBase {

    //Variables that will be used in the command
    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve swerveSubsystem;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private int slowMode;
    private double trans_multiplier;
    private double rot_multiplier;

    //Control Stuff for the driver
    public TeleopSwerve(Swerve swerveSubsystem, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop, int slowMode) { //Inputs from RobotContainer
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);

        //Makes the variables defined earlier equal the values from RobotContainer
        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.slowMode = slowMode;

    }

    @Override
    public void execute() {

        if(controller.getRawButton(slowMode)){
            System.out.println("SLOW MODE");
            trans_multiplier = .25;
            rot_multiplier = .5;
        }
        else{
            trans_multiplier = .6;
            rot_multiplier = 1;
        }

        double yAxis = -controller.getRawAxis(translationAxis);  //Flips the controller values as needed and gets the value of each axis
        double xAxis = -controller.getRawAxis(strafeAxis);
        double rAxis = -controller.getRawAxis(rotationAxis);
        
        // Apply deadbands from constants file to each axis
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis; //If the absolute value of the stick value is less than the deadband as defined in constants, the Axis=0
        //Otherwise if the stick value is greater than the deadband, Axis = the stick value
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;
        //Syntax for this stuff is `variable = TheBoolean ? ifTrue : ifFalse` ^^^^^

        translation = new Translation2d(trans_multiplier*yAxis, trans_multiplier*xAxis).times(Constants.Swerve.maxSpeed); //Adds option for speed multipler defined in constants
        rotation = rot_multiplier * rAxis * Constants.Swerve.maxAngularVelocity;
        swerveSubsystem.drive(translation, rotation, fieldRelative, openLoop);
    }
}
