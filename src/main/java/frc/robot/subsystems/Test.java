package frc.robot.subsystems;


import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.devices.Motor;

public class Test extends SubsystemBase {
//    private static Motor motor;

    private final static Test INSTANCE = new Test();

    public static Test getInstance() {
        return INSTANCE;
    }

    private Test() {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

//        motor = new Motor(1, "TestMotor", 0.0001, 0, 1, false);
        SmartDashboard.putNumber("/Test/0", 0.0);
    }

    public void setMotor(Measure<Velocity<Angle>> velocity) {
//        motor.setVelocity(velocity);
        SmartDashboard.putNumber("/Test/0", velocity.in(Units.RPM));
    }
}

