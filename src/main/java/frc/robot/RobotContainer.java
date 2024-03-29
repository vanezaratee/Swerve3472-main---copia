package frc.robot;

import java.util.List;



import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.joy;
import frc.robot.subsystems.subswerve;

public class RobotContainer {
        private static final subswerve swerveSubsystem = new subswerve();
        private static final XboxController driveControl = new XboxController(0);

        public RobotContainer(){

                swerveSubsystem.setDefaultCommand(new joy(swerveSubsystem, () -> driveControl.getLeftX(),
                                                () -> driveControl.getLeftY(), () -> driveControl.getRightX()));

                configureButtonBindings();
        }

        private void configureButtonBindings(){}

        public Command getAutonomousCommand(){
                return Commands.print("No autonomous command configured");
        }

}


