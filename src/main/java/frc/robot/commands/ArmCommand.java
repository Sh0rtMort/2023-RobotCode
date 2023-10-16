package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase{

    private ArmSubsystem armSubsystem;
    private double armSpeed;

    // private PIDController winchPIDController = new PIDController(0, 0, 0);
    
    public ArmCommand(ArmSubsystem armSubsystem, double armSpeed) {
        // addRequirements(armSubsystem);
        this.armSubsystem = armSubsystem;
        this.armSpeed = armSpeed;
    }

    @Override
    public void initialize() {
        // armSubsystem.setSpeeds(0);
    }


    @Override
    public void execute() {
        armSubsystem.setSpeeds(armSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.setSpeeds(0);
    }
}

