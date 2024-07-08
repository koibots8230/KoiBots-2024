package frc.robot.devices;

public class MotorDefinition {
    public final int CANID;
    public final double P;
    public final double I;
    public final double D;
    public final double IZone;
    public final double FF;
    public final MotorType motorType;

    public MotorDefinition(int CANID, double P, double I, double D, double IZone, double FF, MotorType motorType) {
        this.CANID = CANID;
        this.P = P;
        this.I = I;
        this.D = D;
        this.IZone = IZone;
        this.FF = FF;
        this.motorType = motorType;
    }

    public MotorDefinition(int CANID, MotorType motorType) {
        this.CANID = CANID;
        this.P = 0.0001;
        this.I = 0;
        this.D = 1;
        this.IZone = 0;
        this.FF = 0;
        this.motorType = motorType;
    }

    public enum MotorType {
        CANSPARKMAX_BRUSHLESS,
        CANSPARKMAX_BRUSHED
    }
}
