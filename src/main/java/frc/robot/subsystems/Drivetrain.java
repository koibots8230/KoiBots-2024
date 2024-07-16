package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorFactory;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;
    private final AHRS gyro;
    private final StructArrayPublisher<SwerveModuleState> publisherReal;
    private final StructArrayPublisher<SwerveModuleState> publisherSetpoint;
    private final StructPublisher<Pose2d> publisherPose;

    public Drivetrain(boolean isReal) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        frontLeftModule = new SwerveModule(isReal, Constants.Drivetrain.Drive.ID_FRONT_LEFT,
                Constants.Drivetrain.Turn.ID_FRONT_LEFT);
        frontRightModule = new SwerveModule(isReal, Constants.Drivetrain.Drive.ID_FRONT_RIGHT,
                Constants.Drivetrain.Turn.ID_FRONT_RIGHT);
        backLeftModule = new SwerveModule(isReal, Constants.Drivetrain.Drive.ID_BACK_LEFT,
                Constants.Drivetrain.Turn.ID_BACK_LEFT);
        backRightModule = new SwerveModule(isReal, Constants.Drivetrain.Drive.ID_BACK_RIGHT,
                Constants.Drivetrain.Turn.ID_BACK_RIGHT);

        gyro = new AHRS();
        kinematics = new SwerveDriveKinematics(
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(2), Constants.Robot.ROBOT_WIDTH.divide(2)),
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(2), Constants.Robot.ROBOT_WIDTH.divide(-2)),
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(-2), Constants.Robot.ROBOT_WIDTH.divide(2)),
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(-2), Constants.Robot.ROBOT_WIDTH.divide(-2))
        );
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), getModulePositions());

        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroGyro();
            } catch (Exception ignore) {}
        }).start();

        publisherReal = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/Swerve/State", SwerveModuleState.struct).publish();
        publisherSetpoint = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/Swerve/TargetUnoptimized", SwerveModuleState.struct).publish();
        publisherPose = NetworkTableInstance.getDefault()
                .getStructTopic("/Swerve/Pose", Pose2d.struct).publish();
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), getModulePositions());
        publisherReal.set(getModuleStates());
        publisherPose.set(odometry.getPoseMeters());
    }

    public void drive(double x, double y, double r) {
        x *= -2;
        y *= 2;
        r *= 4;
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(y, x, r));
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);
        publisherSetpoint.set(states);
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    private SwerveModule getModule(SwerveModules module) {
        return switch (module) {
            default -> frontLeftModule;
            case frontRight -> frontRightModule;
            case backLeft -> backLeftModule;
            case backRight -> backRightModule;
        };
    }

    public enum SwerveModules {
        frontLeft,
        frontRight,
        backLeft,
        backRight
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    private static class SwerveModule {
        private final Motor driveMotor;
        private final Motor turnMotor;

        public SwerveModule(boolean isReal, int driveID, int turnID) {
            driveMotor = MotorFactory.get().create(
                    driveID,
                    Constants.Drivetrain.Drive.P,
                    Constants.Drivetrain.Drive.I,
                    Constants.Drivetrain.Drive.D,
                    Constants.Drivetrain.Drive.FF,
                    1 / Constants.Drivetrain.Drive.ENCODER_VELOCITY_FACTOR * 60,
                    1 / Constants.Drivetrain.Drive.ENCODER_POSITION_FACTOR * 60,
                    isReal
            );
            turnMotor = MotorFactory.get().create(
                    turnID,
                    Constants.Drivetrain.Turn.P,
                    Constants.Drivetrain.Turn.I,
                    Constants.Drivetrain.Turn.D,
                    Constants.Drivetrain.Turn.FF,
                    1 / Constants.Drivetrain.Turn.ENCODER_VELOCITY_FACTOR * 60,
                    1 / Constants.Drivetrain.Turn.ENCODER_POSITION_FACTOR * 60,
                    isReal
            );
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(
                    driveMotor.getVelocity().in(Units.RPM), new Rotation2d(turnMotor.getPosition()));
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(
                    driveMotor.getPosition().in(Units.Rotations), new Rotation2d(turnMotor.getPosition()));
        }

        public void setState(SwerveModuleState state) {
            driveMotor.setVelocity(Units.RPM.of(state.speedMetersPerSecond));
            turnMotor.setPosition(Units.Radians.of(state.angle.getRadians()));
        }
    }
}

