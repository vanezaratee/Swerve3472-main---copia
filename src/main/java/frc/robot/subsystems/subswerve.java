package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class subswerve extends SubsystemBase {

    // kinematicas ////////////////////////////////////////////////////////////

    // constantes //
    public static final double kTrackWidth = Units.inchesToMeters(18.89147); //23.25189
    // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(23.25189); //18.89147
    // Distance between front and back wheels

    // plano cartesiano //
    modswerve frontLeft, frontRight, backLeft, backRight;
    static Translation2d fLTranslation = new Translation2d((-kWheelBase / 2), (kTrackWidth / 2)); //front left (back left)   
    static Translation2d fRTranslation = new Translation2d((kWheelBase / 2), (kTrackWidth / 2)); //front right (es front left)
    static Translation2d bLTranslation = new Translation2d((-kWheelBase / 2), (-kTrackWidth / 2)); //back left (back right)
    static Translation2d bRTranslation = new Translation2d((kWheelBase / 2), (-kTrackWidth / 2)); //back right (front right)
    

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(fLTranslation, 
                    fRTranslation, bLTranslation, bRTranslation);

    public subswerve(){
        frontLeft = new modswerve(
            DriveConstants.kFrontLeftDriveMotorPort,
            DriveConstants.kFrontLeftTurningMotorPort,
            DriveConstants.kFrontLeftDriveEncoderReversed,
            DriveConstants.kFrontLeftTurningEncoderReversed,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed); 

        frontRight = new modswerve(
            DriveConstants.kFrontRightDriveMotorPort,
            DriveConstants.kFrontRightTurningMotorPort,
            DriveConstants.kFrontRightDriveEncoderReversed,
            DriveConstants.kFrontRightTurningEncoderReversed,
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort,
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed); 

        backLeft = new modswerve(
            DriveConstants.kBackLeftDriveMotorPort,
            DriveConstants.kBackLeftTurningMotorPort,
            DriveConstants.kBackLeftDriveEncoderReversed,
            DriveConstants.kBackLeftTurningEncoderReversed,
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort,
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed); 

        backRight = new modswerve(
            DriveConstants.kBackRightDriveMotorPort,
            DriveConstants.kBackRightTurningMotorPort,
            DriveConstants.kBackRightDriveEncoderReversed,
            DriveConstants.kBackRightTurningEncoderReversed,
            DriveConstants.kBackRightDriveAbsoluteEncoderPort,
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed); 
    }

    // odometria ///////////////////////////////////////////////////////

    SwerveModulePosition[] positions = {frontLeft.gSwerveModulePosition(),
        frontRight.gSwerveModulePosition(), backLeft.gSwerveModulePosition(), backRight.gSwerveModulePosition()};

    SwerveDriveOdometry odometer = new SwerveDriveOdometry(kDriveKinematics, 
                        getRotation2d(), positions, new Pose2d(0, 0, getRotation2d()));

    // kinematicas ///////////////////////////////////////////////////

    public void setChassisSpeeds(double xSpeed, double ySpeed, double zSpeed){
        SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
        setModuleStates(states);
    }

    private AHRS gyro = new AHRS(SPI.Port.kMXP);

    public void setFieldOrientedSpeed(double xSpeed, double ySpeed, double zSpeed){
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation2d());

        SwerveModuleState[] states = kDriveKinematics.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.kTeleDriveMaxSpeedMetersPerSecond); // change

        setModuleStates(states);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        frontLeft.setDesiredState(states[0]);
        frontRight.setDesiredState(states[1]);
        backLeft.setDesiredState(states[2]);
        backRight.setDesiredState(states[3]);  
   }

    public double getAngle(){
        return gyro.getAngle();
    }

    public Rotation2d getRotation2d(){
        return new Rotation2d(getAngle());
    }

    public Pose2d getPose2d(){
        return odometer.getPoseMeters();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("gyro angle", getAngle());
        SmartDashboard.putNumber("pose X", odometer.getPoseMeters().getX());

        // actualizacion de las posiciones (odometria) //
        positions[0] = frontLeft.gSwerveModulePosition();
        positions[1] = frontRight.gSwerveModulePosition();
        positions[2] = backLeft.gSwerveModulePosition();
        positions[3] = backRight.gSwerveModulePosition();

        odometer.update(getRotation2d(), positions);
    }
}
