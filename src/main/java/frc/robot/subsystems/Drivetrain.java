package frc.robot.subsystems;


import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.Gyro;

public class Drivetrain extends SubsystemBase {
    public static SwerveModule frontLeft;
    public static SwerveModule frontRight;
    public static SwerveModule backLeft;
    public static SwerveModule backRight;
    private static SwerveDriveKinematics kinematics;
    private static SwerveDriveOdometry odometry;
    private static Gyro gyro;

    private final static Drivetrain INSTANCE = new Drivetrain();

    public static Drivetrain getInstance() {
        return INSTANCE;
    }

    private Drivetrain() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        frontLeft = new SwerveModule(Constants.Drivetrain.Drive.ID_FRONT_LEFT,
                Constants.Drivetrain.Turn.ID_FRONT_LEFT);
        frontRight = new SwerveModule(Constants.Drivetrain.Drive.ID_FRONT_RIGHT,
                Constants.Drivetrain.Turn.ID_FRONT_RIGHT);
        backLeft = new SwerveModule(Constants.Drivetrain.Drive.ID_BACK_LEFT,
                Constants.Drivetrain.Turn.ID_BACK_LEFT);
        backRight = new SwerveModule(Constants.Drivetrain.Drive.ID_BACK_RIGHT,
                Constants.Drivetrain.Turn.ID_BACK_RIGHT);
        kinematics = new SwerveDriveKinematics(
                new Translation2d(
                        Constants.Robot.ROBOT_LENGTH.divide(2),
                        Constants.Robot.ROBOT_WIDTH.divide(2)),
                new Translation2d(
                        Constants.Robot.ROBOT_LENGTH.divide(2),
                        Constants.Robot.ROBOT_WIDTH.divide(-2)),
                new Translation2d(
                        Constants.Robot.ROBOT_LENGTH.divide(-2),
                        Constants.Robot.ROBOT_WIDTH.divide(2)),
                new Translation2d(
                        Constants.Robot.ROBOT_LENGTH.divide(-2),
                        Constants.Robot.ROBOT_WIDTH.divide(-2))
        );
        gyro = new Gyro();
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), getModulePositions());
    }

    @Override
    public void periodic() {
        odometry.update(new Rotation2d(gyro.getYaw()), getModulePositions());
    }

    @Override
    public void simulationPeriodic() {
        odometry.update(new Rotation2d(gyro.getYaw()), getModulePositions());
    }

    public void set(Measure<Velocity<Distance>> x, Measure<Velocity<Distance>> y,
                    Measure<Velocity<Angle>> rot, boolean fieldOriented) {
        if (fieldOriented) {
            double tempX = x.in(Units.MetersPerSecond);
            double tempY = y.in(Units.MetersPerSecond);
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

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[]{
                frontLeft.getCurrentState(),
                frontRight.getCurrentState(),
                backLeft.getCurrentState(),
                backRight.getCurrentState()
        };
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void setOdometry(Pose2d pose) {
        odometry.resetPosition(new Rotation2d(gyro.getYaw()), getModulePositions(), pose);
    }

    public class SwerveModule {
        private static CANSparkMax driveMotor;
        private static CANSparkMax turnMotor;
        private static RelativeEncoder driveEncoder;
        private static AbsoluteEncoder turnEncoder;
        private static SparkPIDController drivePID;
        private static PIDController turnPID;
        private static SwerveModuleState state;

        private SwerveModule(int driveCANID, int turnCANID) {
            driveMotor = new CANSparkMax(driveCANID, CANSparkLowLevel.MotorType.kBrushless);
            turnMotor = new CANSparkMax(turnCANID, CANSparkLowLevel.MotorType.kBrushless);
            driveEncoder = driveMotor.getEncoder();
            turnEncoder = turnMotor.getAbsoluteEncoder();
            drivePID = driveMotor.getPIDController();
            drivePID.setP(Constants.Drivetrain.Drive.P);
            drivePID.setI(Constants.Drivetrain.Drive.I);
            drivePID.setD(Constants.Drivetrain.Drive.D);
            drivePID.setFF(Constants.Drivetrain.Drive.FF);
            drivePID.setIZone(Constants.Drivetrain.Drive.I_ZONE);
            turnPID = new PIDController(
                    Constants.Drivetrain.Turn.P, Constants.Drivetrain.Turn.I, Constants.Drivetrain.Turn.D);
            turnPID.enableContinuousInput(-Math.PI, Math.PI);
        }

        public Measure<Velocity<Angle>> DVToAV (Measure<Velocity<Distance>> velocity) {
            return Units.RotationsPerSecond.of(velocity.in(Units.MetersPerSecond) /
                    Constants.Drivetrain.Drive.WHEEL_DIAMETER.in(Units.Meters) * Constants.Drivetrain.Drive.GEARING);
        }

        public Measure<Velocity<Distance>> AVToDV (Measure<Velocity<Angle>> velocity) {
            return Units.MetersPerSecond.of(velocity.in(Units.RotationsPerSecond) *
                    Constants.Drivetrain.Drive.WHEEL_DIAMETER.in(Units.Meters) / Constants.Drivetrain.Drive.GEARING);
        }

        public Measure<Angle> DToR (Measure<Distance> distance) {
            return Units.Rotations.of(distance.in(Units.Meters) /
                    Constants.Drivetrain.Drive.WHEEL_DIAMETER.in(Units.Meters) * Constants.Drivetrain.Drive.GEARING);
        }

        public Measure<Distance> RToD (Measure<Angle> rotations) {
            return Units.Meters.of(rotations.in(Units.Rotations) *
                    Constants.Drivetrain.Drive.WHEEL_DIAMETER.in(Units.Meters) / Constants.Drivetrain.Drive.GEARING);
        }

        public Measure<Velocity<Distance>> getVelocity() {
            return AVToDV(Units.RPM.of(driveEncoder.getVelocity()));
        }

        public Measure<Angle> getAngle() {
            return Units.Radians.of(turnEncoder.getPosition() / (Math.PI * 2));
        }

        public Measure<Distance> getDistance() {
            return RToD(Units.Rotations.of(driveEncoder.getPosition()));
        }

        public SwerveModuleState getTargetState() {
            return state;
        }

        public SwerveModuleState getCurrentState() {
            return new SwerveModuleState(getVelocity(), new Rotation2d(getAngle()));
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(getDistance(), new Rotation2d(getAngle()));
        }

        public void setStateNoOptimize(SwerveModuleState state) {
            if(Math.abs(state.speedMetersPerSecond) < 0.001) {
                stop();
                return;
            }

            SwerveModule.state = state;

            double turnFeedback = turnPID.calculate(getAngle().in(Units.Radians), state.angle.getRadians());
            turnMotor.setVoltage(turnFeedback + Math.signum(turnFeedback) * Constants.Drivetrain.Turn.S);

            drivePID.setReference(DVToAV(Units.MetersPerSecond.of(state.speedMetersPerSecond)).in(Units.RPM),
                    CANSparkBase.ControlType.kVelocity);
        }

        public void setState(SwerveModuleState state) {
            SwerveModuleState optimizedState = SwerveModuleState.optimize(state, new Rotation2d(getAngle()));
            optimizedState.speedMetersPerSecond *= Math.cos( // Cosine Optimization
                    Math.abs(optimizedState.angle.getRadians() - getAngle().in(Units.Radians)));

            setStateNoOptimize(optimizedState);
        }

        public void resetAbsoluteEncoder() {
            turnEncoder.setZeroOffset(turnEncoder.getPosition() % 1);
        }

        public void stop() {
            driveMotor.setVoltage(0);
            turnMotor.setVoltage(0);
        }
    }
}

