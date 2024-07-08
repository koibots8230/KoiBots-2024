package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.Gyro;
import frc.robot.devices.Motor;
import frc.robot.devices.SwerveModule;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final SwerveDriveKinematics kinematics;

    private final static Drivetrain INSTANCE = new Drivetrain();

    public static Drivetrain getInstance() {
        return INSTANCE;
    }

    private final static Gyro gyro = new Gyro();

    public static Gyro getGyro() {
        return gyro;
    }

    private Drivetrain() {
        frontLeft = new SwerveModule(
                new Motor(Constants.Motors.FrontLeftDrive),
                new Motor(Constants.Motors.FrontLeftTurn));
        frontRight = new SwerveModule(
                new Motor(Constants.Motors.FrontRightDrive),
                new Motor(Constants.Motors.FrontRightTurn));
        backLeft = new SwerveModule(
                new Motor(Constants.Motors.BackLeftDrive),
                new Motor(Constants.Motors.BackLeftTurn));
        backRight = new SwerveModule(
                new Motor(Constants.Motors.BackRightDrive),
                new Motor(Constants.Motors.BackRightTurn));
        kinematics = new SwerveDriveKinematics(
                new Translation2d(Units.Inches.of(10.75), Units.Inches.of(10.75)),
                new Translation2d(Units.Inches.of(10.75), Units.Inches.of(-10.75)),
                new Translation2d(Units.Inches.of(-10.75), Units.Inches.of(10.75)),
                new Translation2d(Units.Inches.of(-10.75), Units.Inches.of(-10.75))
        );
    }

    public void set(Measure<Velocity<Distance>> x, Measure<Velocity<Distance>> y, Measure<Velocity<Angle>> rot) {
        if (Constants.Operation.FIELD_ORIENTED) {
            double tempX = x.in(Units.MetersPerSecond);
            double tempY = x.in(Units.MetersPerSecond);
            double theta = gyro.getYaw().in(Units.Radians);
            x = Units.MetersPerSecond.of(tempX * Math.cos(theta) - tempY * Math.sin(theta));
            y = Units.MetersPerSecond.of(tempY * Math.cos(theta) + tempX * Math.sin(theta));
        }

        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(new ChassisSpeeds(x, y, rot));

        frontLeft.setState(swerveModuleStates[0]);
        frontRight.setState(swerveModuleStates[1]);
        backLeft.setState(swerveModuleStates[2]);
        backRight.setState(swerveModuleStates[3]);


    }
}

