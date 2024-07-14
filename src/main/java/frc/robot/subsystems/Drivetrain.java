package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(-2), Constants.Robot.ROBOT_WIDTH.divide(2)),
                new Translation2d(Constants.Robot.ROBOT_LENGTH.divide(2), Constants.Robot.ROBOT_WIDTH.divide(-2)),
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
                .getStructArrayTopic("/SwerveStatesReal", SwerveModuleState.struct).publish();
        publisherSetpoint = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/SwerveStatesTargetUnoptimized", SwerveModuleState.struct).publish();
    }

    @Override
    public void periodic() {
        odometry.update(getYaw(), getModulePositions());
        publisherReal.set(getModuleStates());
    }

    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
                frontLeftModule.getPosition(),
                frontRightModule.getPosition(),
                backLeftModule.getPosition(),
                backRightModule.getPosition()
        };
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
                frontLeftModule.getState(),
                frontRightModule.getState(),
                backLeftModule.getState(),
                backRightModule.getState()
        };
    }

    public void drive(double x, double y, double r) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(new ChassisSpeeds(y, -x, r));
        frontLeftModule.setState(states[0]);
        frontRightModule.setState(states[1]);
        backLeftModule.setState(states[2]);
        backRightModule.setState(states[3]);

        SmartDashboard.putNumber("x", x);
        SmartDashboard.putNumber("y", y);
        SmartDashboard.putNumber("r", r);
        publisherSetpoint.set(states);
    }

    public void setModule(SwerveModuleState state, SwerveModules module) {
        getModule(module).setStateNoOptimize(state);
        SmartDashboard.putNumber(module.toString(), SmartDashboard.getNumber(module.toString(), 0) + 1);
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
            driveEncoder.setPositionConversionFactor(Constants.Drivetrain.Drive.ENCODER_POSITION_FACTOR);
            turnEncoder.setPositionConversionFactor(Constants.Drivetrain.Turn.ENCODER_POSITION_FACTOR);
            driveEncoder.setVelocityConversionFactor(Constants.Drivetrain.Drive.ENCODER_VELOCITY_FACTOR);
            turnEncoder.setVelocityConversionFactor(Constants.Drivetrain.Turn.ENCODER_VELOCITY_FACTOR);
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

