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

    public Test(boolean isReal) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        this.isReal = isReal;
        testMotor = new CANSparkMax(Constants.TestSubsystem.CANID, CANSparkLowLevel.MotorType.kBrushless);
        if (!isReal) REVPhysicsSim.getInstance().addSparkMax(testMotor, DCMotor.getNEO(1));
        PIDController = testMotor.getPIDController();
        PIDController.setP(Constants.TestSubsystem.P);
        PIDController.setI(Constants.TestSubsystem.I);
        PIDController.setD(Constants.TestSubsystem.D);
        PIDController.setFF(Constants.TestSubsystem.FF);
        PIDController.setIZone(Constants.TestSubsystem.I_ZONE);
        relativeEncoder = testMotor.getEncoder();
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        PIDController.setReference(velocity.in(Units.RPM), CANSparkBase.ControlType.kVelocity);
    }
}

