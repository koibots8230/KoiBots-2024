package frc.robot.devices;


import com.revrobotics.*;
import edu.wpi.first.units.*;


public class Motor {
    private static MotorDefinition definition;
    private static Measure<Velocity<Angle>> velocity;
    private static Measure<Voltage> voltage;
    private static CANSparkMax canSparkMax;
    private static SparkPIDController sparkPIDController;

    public Motor(frc.robot.devices.MotorDefinition motor) {
        Motor.definition = motor;

        switch (definition.motorType) {
            case CANSPARKMAX_BRUSHLESS:
                Motor.canSparkMax = new CANSparkMax(Motor.definition.CANID, CANSparkLowLevel.MotorType.kBrushless);
                return;
            case CANSPARKMAX_BRUSHED:
                Motor.canSparkMax = new CANSparkMax(Motor.definition.CANID, CANSparkLowLevel.MotorType.kBrushed);
                return;
            default:
                Motor.canSparkMax = new CANSparkMax(Motor.definition.CANID, CANSparkLowLevel.MotorType.kBrushless);
        }

        Motor.encoder = Motor.canSparkMax.getEncoder();
        Motor.sparkPIDController = Motor.canSparkMax.getPIDController();
        Motor.sparkPIDController.setP(Motor.definition.P);
        Motor.sparkPIDController.setI(Motor.definition.I);
        Motor.sparkPIDController.setD(Motor.definition.D);
        Motor.sparkPIDController.setIZone(Motor.definition.IZone);
        Motor.sparkPIDController.setFF(Motor.definition.FF);
    }

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        Motor.voltage = null;
        Motor.velocity = velocity;
        Motor.sparkPIDController.setReference(Motor.velocity.in(Units.RPM), CANSparkBase.ControlType.kVelocity);
    }

    public void setVoltage(Measure<Voltage> voltage) {
        Motor.velocity = null;
        Motor.voltage = voltage;
        Motor.canSparkMax.setVoltage(Motor.voltage.in(Units.Volts));
    }

    public RelativeEncoder getRelativeEncoder() {
        return Motor.canSparkMax.getEncoder();
    }

    public AbsoluteEncoder getAbsoluteEncoder() {
        return Motor.canSparkMax.getAbsoluteEncoder();
    }
}
