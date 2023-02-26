package frc.robot.commands;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
public class Balance extends CommandBase {

    private Swerve jeffry;
    private Pigeon2 gyro;

    private Swerve swerve() 
        public Balance(Swerve jeffery){
            this.jeffry = jeffery;
    }
    private double R; //Picth of the robot
    private static final int T; //Tolarence angle
    private double X; //= get x
    private double Y; //= get y
    @Override
    public void execute() {
        R = jeffry.getPitch();
        System.out.println(R,x,y);
        if(R < 0 - T){
            System.out.println(x,y-movement)
            //movement (set translation)
            return R = 
        }
        if(R > 0 + T){
            System.out.println(x,y+movement)
            //movement (set translation)
        }
    }
}

