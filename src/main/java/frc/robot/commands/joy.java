package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.subswerve;

public class joy extends Command{
    subswerve swerveSubsystem;
    Supplier<Double> xSpeed, ySpeed, zSpeed;


    public joy(subswerve swerveSubsystem, Supplier<Double> xSpeed, 
            Supplier<Double> ySpeed, Supplier<Double> zSpeed){
                
        addRequirements(swerveSubsystem);        
        this.swerveSubsystem = swerveSubsystem;
            
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.zSpeed = zSpeed;
    }


    @Override
    public void execute(){
        swerveSubsystem.setFieldOrientedSpeed(xSpeed.get(), ySpeed.get(), zSpeed.get());
    
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}