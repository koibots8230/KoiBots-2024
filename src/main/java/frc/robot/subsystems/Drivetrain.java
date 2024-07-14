package frc.robot.subsystems;


import com.revrobotics.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
    public Drivetrain(boolean isReal) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

    }

    private static class SwerveModule {
        private final CANSparkMax driveMotor;
        private final CANSparkMax turnMotor;

        private final RelativeEncoder driveEncoder;
        private final AbsoluteEncoder turnEncoder;

        private final SparkPIDController driveController;
        private final SparkPIDController turnController;

        private double chassisAngularOffset = 0;
        private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

        public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
            driveMotor = new CANSparkMax(drivingCANId, CANSparkLowLevel.MotorType.kBrushless);
            turnMotor = new CANSparkMax(turningCANId, CANSparkLowLevel.MotorType.kBrushless);

            driveMotor.restoreFactoryDefaults();
            turnMotor.restoreFactoryDefaults();

            driveEncoder = driveMotor.getEncoder();
            turnEncoder = turnMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
            driveController = driveMotor.getPIDController();
            turnController = turnMotor.getPIDController();
            driveController.setFeedbackDevice(driveEncoder);
            turnController.setFeedbackDevice(turnEncoder);

            driveEncoder.setPositionConversionFactor(Constants.Drivetrain.Module.DrivingEncoderPositionFactor);
            driveEncoder.setVelocityConversionFactor(Constants.Drivetrain.Module.DrivingEncoderVelocityFactor);

            turnEncoder.setPositionConversionFactor(Constants.Drivetrain.Module.TurningEncoderPositionFactor);
            turnEncoder.setVelocityConversionFactor(Constants.Drivetrain.Module.TurningEncoderVelocityFactor);

            turnEncoder.setInverted(Constants.Drivetrain.Module.TurningEncoderInverted);

            turnController.setPositionPIDWrappingEnabled(true);
            turnController.setPositionPIDWrappingMinInput(
                    Constants.Drivetrain.Module.TurningEncoderPositionPIDMinInput);
            turnController.setPositionPIDWrappingMaxInput(
                    Constants.Drivetrain.Module.TurningEncoderPositionPIDMaxInput);

            driveController.setP(Constants.Drivetrain.Module.DrivingP);
            driveController.setI(Constants.Drivetrain.Module.DrivingI);
            driveController.setD(Constants.Drivetrain.Module.DrivingD);
            driveController.setFF(Constants.Drivetrain.Module.DrivingFF);
            driveController.setOutputRange(Constants.Drivetrain.Module.DrivingMinOutput,
                    Constants.Drivetrain.Module.DrivingMaxOutput);

            turnController.setP(Constants.Drivetrain.Module.TurningP);
            turnController.setI(Constants.Drivetrain.Module.TurningI);
            turnController.setD(Constants.Drivetrain.Module.TurningD);
            turnController.setFF(Constants.Drivetrain.Module.TurningFF);
            turnController.setOutputRange(Constants.Drivetrain.Module.TurningMinOutput,
                    Constants.Drivetrain.Module.TurningMaxOutput);

            driveMotor.setIdleMode(Constants.Drivetrain.Module.DrivingMotorIdleMode);
            turnMotor.setIdleMode(Constants.Drivetrain.Module.TurningMotorIdleMode);
            driveMotor.setSmartCurrentLimit(Constants.Drivetrain.Module.DrivingMotorCurrentLimit);
            turnMotor.setSmartCurrentLimit(Constants.Drivetrain.Module.TurningMotorCurrentLimit);

            driveMotor.burnFlash();
            turnMotor.burnFlash();

            this.chassisAngularOffset = chassisAngularOffset;
            desiredState.angle = new Rotation2d( turnEncoder.getPosition());
            driveEncoder.setPosition(0);
        }

        public SwerveModuleState getState() {
            return new SwerveModuleState(driveEncoder.getVelocity(),
                    new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
        }

        public SwerveModulePosition getPosition() {
            return new SwerveModulePosition(driveEncoder.getPosition(),
                    new Rotation2d(turnEncoder.getPosition() - chassisAngularOffset));
        }

        public void setDesiredState(SwerveModuleState desiredState) {
            SwerveModuleState correctedDesiredState = new SwerveModuleState();
            correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
            correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

            SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                    new Rotation2d(turnEncoder.getPosition()));

            driveController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
            turnController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);

            this.desiredState = desiredState;
        }

        public void resetEncoders() {
            driveEncoder.setPosition(0);
        }
    }
}

