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
import edu.wpi.first.units.Units;
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
        private CANSparkMax driveMotor;
        private CANSparkMax turnMotor;
        private SparkPIDController driveController;
        private SparkPIDController turnController;
        private RelativeEncoder driveEncoder;
        private AbsoluteEncoder turnEncoder;

        public SwerveModule(boolean isReal, int driveCANID, int turnCANID) {
            driveMotor = new CANSparkMax(driveCANID, CANSparkLowLevel.MotorType.kBrushless);
            turnMotor = new CANSparkMax(turnCANID, CANSparkLowLevel.MotorType.kBrushless);
            if (isReal) {
                REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
                REVPhysicsSim.getInstance().addSparkMax(turnMotor, DCMotor.getNEO(1));
            }
            driveMotor.restoreFactoryDefaults();
            turnMotor.restoreFactoryDefaults();
            driveController = driveMotor.getPIDController();
            turnController = turnMotor.getPIDController();
            driveEncoder = driveMotor.getEncoder();
            turnEncoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
            driveController.setFeedbackDevice(driveEncoder);
            turnController.setFeedbackDevice(turnEncoder);
            driveEncoder.setPositionConversionFactor(Constants.Drivetrain.Module.DriveEncoderPositionFactor);
            turnEncoder.setPositionConversionFactor(Constants.Drivetrain.Module.TurnEncoderPositionFactor);
            driveEncoder.setVelocityConversionFactor(Constants.Drivetrain.Module.DriveEncoderVelocityFactor);
            turnEncoder.setVelocityConversionFactor(Constants.Drivetrain.Module.TurnEncoderVelocityFactor);
            turnEncoder.setInverted(Constants.Drivetrain.Turn.INVERT);
            turnController.setPositionPIDWrappingEnabled(true);
            turnController.setPositionPIDWrappingMinInput(Constants.Drivetrain.Turn.MIN_IN);
            turnController.setPositionPIDWrappingMaxInput(Constants.Drivetrain.Turn.MAX_IN);
            driveController.setP(Constants.Drivetrain.Drive.P);
            driveController.setI(Constants.Drivetrain.Drive.I);
            driveController.setD(Constants.Drivetrain.Drive.D);
            driveController.setFF(Constants.Drivetrain.Drive.FF);
            driveController.setOutputRange(Constants.Drivetrain.Drive.MIN_OUT, Constants.Drivetrain.Drive.MAX_OUT);
            turnController.setP(Constants.Drivetrain.Turn.P);
            turnController.setI(Constants.Drivetrain.Turn.I);
            turnController.setD(Constants.Drivetrain.Turn.D);
            turnController.setFF(Constants.Drivetrain.Turn.FF);
            turnController.setOutputRange(Constants.Drivetrain.Turn.MIN_OUT, Constants.Drivetrain.Turn.MAX_OUT);
            driveMotor.setIdleMode(Constants.Drivetrain.Drive.IDLE_MODE);
            turnMotor.setIdleMode(Constants.Drivetrain.Turn.IDLE_MODE);
            driveMotor.setSmartCurrentLimit((int) Constants.Drivetrain.Drive.CURRENT_LIMIT.in(Units.Amps));
            turnMotor.setSmartCurrentLimit((int) Constants.Drivetrain.Turn.CURRENT_LIMIT.in(Units.Amps));
            driveMotor.burnFlash();
            turnMotor.burnFlash();
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(turnEncoder.getPosition()));
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(turnEncoder.getPosition()));
        }

        public void setStateNoOptimize(SwerveModuleState state) {
            driveController.setReference(state.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
            turnController.setReference(state.angle.getRadians(), CANSparkBase.ControlType.kPosition);
        }

        public void setState(SwerveModuleState state) {
            SwerveModuleState optimizedState =
                    SwerveModuleState.optimize(state, new Rotation2d(turnEncoder.getPosition()));
            optimizedState.speedMetersPerSecond *=
                    optimizedState.angle.minus(new Rotation2d(turnEncoder.getPosition())).getCos();
        }


        public void zeroTurnEncoder() {
            turnEncoder.setZeroOffset(0);
            turnEncoder.setZeroOffset(turnEncoder.getPosition());
        }
    }
}

