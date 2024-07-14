package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    private SwerveModule frontLeftModule;
    private SwerveModule frontRightModule;
    private SwerveModule backLeftModule;
    private SwerveModule backRightModule;
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private AHRS gyro;

    public Drivetrain(boolean isReal) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        int moduleRotationOffset = 0;
        frontLeftModule = new SwerveModule(isReal, Constants.Drivetrain.Drive.ID_FRONT_LEFT,
                Constants.Drivetrain.Turn.ID_FRONT_LEFT, Math.PI / 2 * 0 + moduleRotationOffset);
        frontRightModule = new SwerveModule(isReal, Constants.Drivetrain.Drive.ID_FRONT_RIGHT,
                Constants.Drivetrain.Turn.ID_FRONT_RIGHT, Math.PI / 2 * 1 + moduleRotationOffset);
        backLeftModule = new SwerveModule(isReal, Constants.Drivetrain.Drive.ID_BACK_LEFT,
                Constants.Drivetrain.Turn.ID_BACK_LEFT, Math.PI / 2 * 3 + moduleRotationOffset);
        backRightModule = new SwerveModule(isReal, Constants.Drivetrain.Drive.ID_BACK_RIGHT,
                Constants.Drivetrain.Turn.ID_BACK_RIGHT, Math.PI / 2 * 2 + moduleRotationOffset);

        gyro = new AHRS();
        kinematics = new SwerveDriveKinematics(
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(2), Constants.Robot.ROBOT_WIDTH.divide(2)),
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(-2), Constants.Robot.ROBOT_WIDTH.divide(2)),
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(2), Constants.Robot.ROBOT_WIDTH.divide(-2)),
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(-2), Constants.Robot.ROBOT_WIDTH.divide(-2))
        );
        odometry = new SwerveDriveOdometry();
    }

    private static class SwerveModule {
        private SwerveModule() {

        }
    }
}

