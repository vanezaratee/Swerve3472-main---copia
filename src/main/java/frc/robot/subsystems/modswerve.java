package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.fasterxml.jackson.databind.jsontype.impl.SubTypeValidator;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class modswerve extends SubsystemBase{

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public modswerve(int driveMotorId, int turningMotorId,
        boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {


        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

    }

    // kinematicas //////////////////////////////////////////////////////

    public void setDesiredState(SwerveModuleState desiredState){
        desiredState = SwerveModuleState.optimize(desiredState, getAngle());

        setSpeed(desiredState);
        setAngle(desiredState);
    }

    public void setSpeed(SwerveModuleState desiredState){
        driveMotor.set(desiredState.speedMetersPerSecond);
    }

    public void setAngle(SwerveModuleState desiredState){
        double pidValue = turningPidController.calculate(getAngle().getRadians(),
                 desiredState.angle.getRadians());
        turningMotor.set(pidValue);
    }

    public Rotation2d getAngle(){
        return Rotation2d.fromRadians(absoluteEncoderOffsetRad); //turning encoder???
    }

    // odometria //////////////////////////////////////////////////////////////

    public SwerveModulePosition gSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    public double getDrivePosition() {
        double position;
        position = driveEncoder.getPosition();
        return position;
    }

}
