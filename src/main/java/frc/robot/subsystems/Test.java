package frc.robot.subsystems;


import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.devices.motor.Motor;
import frc.robot.devices.motor.MotorFactory;

public class Test extends SubsystemBase {
    private final Motor motor;

    public Test(boolean isReal) {
        // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
        //       in the constructor or in the robot coordination class, such as RobotContainer.
        //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
        //       such as SpeedControllers, Encoders, DigitalInputs, etc.

        motor = MotorFactory.get().create(
                Constants.TestSubsystem.CANID,
                Constants.TestSubsystem.P,
                Constants.TestSubsystem.I,
                Constants.TestSubsystem.D,
                Constants.TestSubsystem.FF,
                2,
                2,
                isReal
        );
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        motor.setVelocity(velocity);
    }

    public void setPosition(Measure<Angle> position) {
        motor.setPosition(position);
    }
}

