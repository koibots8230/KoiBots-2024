package frc.robot.subsystems;


import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
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
                .getStructArrayTopic("/Swerve/State", SwerveModuleState.struct).publish();
        publisherSetpoint = NetworkTableInstance.getDefault()
                .getStructArrayTopic("/Swerve/TargetUnoptimized", SwerveModuleState.struct).publish();
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
        private final CANSparkMax driveMotor;
        private final CANSparkMax turnMotor;
        private final SparkPIDController driveController;
        private final SparkPIDController turnController;
        private final RelativeEncoder driveEncoder;
        private final AbsoluteEncoder turnEncoder;
        private final RelativeEncoder turnEncoderSim;
        private final boolean isReal;

        public SwerveModule(boolean isReal, int driveCANID, int turnCANID) {
            this.isReal = isReal;
            driveMotor = new CANSparkMax(driveCANID, CANSparkLowLevel.MotorType.kBrushless);
            turnMotor = new CANSparkMax(turnCANID, CANSparkLowLevel.MotorType.kBrushless);
            if (isReal) {
                REVPhysicsSim.getInstance().addSparkMax(driveMotor, DCMotor.getNEO(1));
                REVPhysicsSim.getInstance().addSparkMax(turnMotor, DCMotor.getNEO(1));
            }

            driveMotor.restoreFactoryDefaults();
            driveController = driveMotor.getPIDController();
            driveEncoder = driveMotor.getEncoder();
            driveController.setFeedbackDevice(driveEncoder);
            driveEncoder.setPositionConversionFactor(Constants.Drivetrain.Drive.ENCODER_POSITION_FACTOR);
            driveEncoder.setVelocityConversionFactor(Constants.Drivetrain.Drive.ENCODER_VELOCITY_FACTOR);
            driveController.setP(Constants.Drivetrain.Drive.P);
            driveController.setI(Constants.Drivetrain.Drive.I);
            driveController.setD(Constants.Drivetrain.Drive.D);
            driveController.setFF(Constants.Drivetrain.Drive.FF);
            driveController.setOutputRange(Constants.Drivetrain.Drive.MIN_OUT, Constants.Drivetrain.Drive.MAX_OUT);
            driveMotor.setIdleMode(Constants.Drivetrain.Drive.IDLE_MODE);
            driveMotor.setSmartCurrentLimit((int) Constants.Drivetrain.Drive.CURRENT_LIMIT.in(Units.Amps));
            driveMotor.burnFlash();

            turnMotor.restoreFactoryDefaults();
            turnController = turnMotor.getPIDController();
            turnEncoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
            turnController.setFeedbackDevice(turnEncoder);
            turnEncoder.setPositionConversionFactor(Constants.Drivetrain.Turn.ENCODER_POSITION_FACTOR);
            turnEncoder.setVelocityConversionFactor(Constants.Drivetrain.Turn.ENCODER_VELOCITY_FACTOR);
            turnEncoder.setInverted(Constants.Drivetrain.Turn.INVERT);
            turnController.setPositionPIDWrappingEnabled(true);
            turnController.setPositionPIDWrappingMinInput(Constants.Drivetrain.Turn.MIN_IN);
            turnController.setPositionPIDWrappingMaxInput(Constants.Drivetrain.Turn.MAX_IN);
            turnController.setP(Constants.Drivetrain.Turn.P);
            turnController.setI(Constants.Drivetrain.Turn.I);
            turnController.setD(Constants.Drivetrain.Turn.D);
            turnController.setFF(Constants.Drivetrain.Turn.FF);
            turnController.setOutputRange(Constants.Drivetrain.Turn.MIN_OUT, Constants.Drivetrain.Turn.MAX_OUT);
            turnMotor.setIdleMode(Constants.Drivetrain.Turn.IDLE_MODE);
            turnMotor.setSmartCurrentLimit((int) Constants.Drivetrain.Turn.CURRENT_LIMIT.in(Units.Amps));
            turnMotor.burnFlash();

            turnEncoderSim = turnMotor.getEncoder();
            turnController.setFeedbackDevice(turnEncoderSim);
            turnEncoderSim.setPositionConversionFactor(Constants.Drivetrain.Turn.ENCODER_POSITION_FACTOR);
            turnEncoderSim.setVelocityConversionFactor(Constants.Drivetrain.Turn.ENCODER_VELOCITY_FACTOR);
            turnEncoder.setInverted(Constants.Drivetrain.Turn.INVERT);
        }

        public Rotation2d getAngle() {
            if (isReal) return new Rotation2d(turnEncoder.getPosition());
            else return new Rotation2d(turnEncoderSim.getPosition());
        }

        public Measure<Distance> getDistance() {
            return Units.Meters.of(driveEncoder.getPosition());
        }

        public Measure<Velocity<Distance>> getVelocity() {
            return Units.MetersPerSecond.of(driveEncoder.getVelocity());
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(getVelocity(), getAngle());
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(getDistance(), getAngle());
        }

        public void setStateNoOptimize(SwerveModuleState state) {
            driveController.setReference(state.speedMetersPerSecond, CANSparkBase.ControlType.kVelocity);
            turnController.setReference(state.angle.getRadians(), CANSparkBase.ControlType.kPosition);
        }

        public void setState(SwerveModuleState state) {
            SwerveModuleState optimizedState =
                    SwerveModuleState.optimize(state, getAngle());
            optimizedState.speedMetersPerSecond *=
                    optimizedState.angle.minus(getAngle()).getCos();
            setStateNoOptimize(state);
        }

        public void zeroTurnEncoder() {
            turnEncoder.setZeroOffset(0);
            turnEncoder.setZeroOffset(turnEncoder.getPosition());
            turnEncoderSim.setPosition(0);
        }
    }
}

