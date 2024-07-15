package frc.robot.subsystems;


import com.revrobotics.*;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Test extends SubsystemBase {
    private final boolean isReal;
    private final CANSparkMax testMotor;
    private final SparkPIDController PIDController;
    private final RelativeEncoder relativeEncoder;
    private final AbsoluteEncoder absoluteEncoder;

    public Test(boolean isReal) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        this.isReal = isReal;
        testMotor = new CANSparkMax(Constants.TestSubsystem.Turn.CANID, CANSparkLowLevel.MotorType.kBrushless);
        if (!isReal) REVPhysicsSim.getInstance().addSparkMax(testMotor, DCMotor.getNEO(1));
        testMotor.restoreFactoryDefaults();
        PIDController = testMotor.getPIDController();
        relativeEncoder = testMotor.getEncoder();
        relativeEncoder.setPositionConversionFactor(Constants.TestSubsystem.Drive.ENCODER_POSITION_FACTOR);
        relativeEncoder.setVelocityConversionFactor(Constants.TestSubsystem.Drive.ENCODER_VELOCITY_FACTOR);
        absoluteEncoder = testMotor.getAbsoluteEncoder();
        absoluteEncoder.setPositionConversionFactor(Constants.TestSubsystem.Drive.ENCODER_POSITION_FACTOR);
        absoluteEncoder.setVelocityConversionFactor(Constants.TestSubsystem.Drive.ENCODER_VELOCITY_FACTOR);
        PIDController.setPositionPIDWrappingEnabled(true);
        PIDController.setPositionPIDWrappingMinInput(Constants.TestSubsystem.Turn.MIN_IN);
        PIDController.setPositionPIDWrappingMaxInput(Constants.TestSubsystem.Turn.MAX_IN);
        PIDController.setP(Constants.TestSubsystem.Turn.P);
        PIDController.setI(Constants.TestSubsystem.Turn.I);
        PIDController.setD(Constants.TestSubsystem.Turn.D);
        PIDController.setFF(Constants.TestSubsystem.Turn.FF);
        PIDController.setOutputRange(Constants.TestSubsystem.Turn.MIN_OUT, Constants.TestSubsystem.Turn.MAX_OUT);
        PIDController.setFeedbackDevice(absoluteEncoder);
        testMotor.setIdleMode(Constants.TestSubsystem.Turn.IDLE_MODE);
        testMotor.setSmartCurrentLimit((int) Constants.TestSubsystem.Turn.CURRENT_LIMIT.in(Units.Amps));
        testMotor.burnFlash();
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        PIDController.setReference(velocity.in(Units.RPM), CANSparkBase.ControlType.kVelocity);
        SmartDashboard.putNumber("Test/VelocityRealRelative", relativeEncoder.getVelocity());
        SmartDashboard.putNumber("Test/VelocityRealAbsolute", absoluteEncoder.getVelocity());
        SmartDashboard.putNumber("Test/VelocityTarget", velocity.in(Units.RPM));
        SmartDashboard.putNumber("Test/PositionRealRelative", relativeEncoder.getPosition());
        SmartDashboard.putNumber("Test/PositionRealAbsolute", absoluteEncoder.getPosition());
    }

    public void setPosition(int position) {
        PIDController.setReference(position, CANSparkBase.ControlType.kPosition);
        SmartDashboard.putNumber("Test/VelocityRealRelative", relativeEncoder.getVelocity());
        SmartDashboard.putNumber("Test/VelocityRealAbsolute", absoluteEncoder.getVelocity());
        SmartDashboard.putNumber("Test/PositionRealRelative", relativeEncoder.getPosition());
        SmartDashboard.putNumber("Test/PositionRealAbsolute", absoluteEncoder.getPosition());
        SmartDashboard.putNumber("Test/PositionTarget", position);
    }
}

