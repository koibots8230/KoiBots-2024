package frc.robot;

import com.revrobotics.CANSparkBase;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {
    public static void test(int id) {
        SmartDashboard.putString("Test/".concat(String.valueOf(id)), "^.^");
    }

    public static void test(int id, String text) {
        SmartDashboard.putString("Test/".concat(String.valueOf(id)), text);
    }

    public static final class Drivetrain {
        public static final class Drive {
            public static final int ID_FRONT_LEFT = 1;
            public static final int ID_FRONT_RIGHT = 3;
            public static final int ID_BACK_LEFT = 5;
            public static final int ID_BACK_RIGHT = 7;
            public static final double P = 0.0001;
            public static final double I = 0;
            public static final double D = 1;
            public static final double FF = 0;
            public static final double MIN_OUT = -1;
            public static final double MAX_OUT = 1;
            public static final CANSparkBase.IdleMode IDLE_MODE = CANSparkBase.IdleMode.kBrake;
            public static final Measure<Current> CURRENT_LIMIT = Units.Amps.of(50);
            public static final int PINION_TEETH = 14;
        }

        public static final class Turn {
            public static final int ID_FRONT_LEFT = 2;
            public static final int ID_FRONT_RIGHT = 4;
            public static final int ID_BACK_LEFT = 6;
            public static final int ID_BACK_RIGHT = 8;
            public static final double P = 0.0001;
            public static final double I = 0;
            public static final double D = 1;
            public static final double FF = 0;
            public static final double MIN_IN = 0;
            public static final double MAX_IN = Module.TurnEncoderPositionFactor;
            public static final double MIN_OUT = -1;
            public static final double MAX_OUT = 1;
            public static final CANSparkBase.IdleMode IDLE_MODE = CANSparkBase.IdleMode.kBrake;
            public static final Measure<Current> CURRENT_LIMIT = Units.Amps.of(20);
            public static final boolean INVERT = true;
        }

        public static final class Module { // Mayhaps stolen from REV :3
            // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
            // This changes the drive speed of the module (a pinion gear with more teeth will result in a
            // robot that drives faster).

            // Invert the turning encoder, since the output shaft rotates in the opposite direction of
            // the steering motor in the MAXSwerve Module.

            // Calculations required for driving motor conversion factors and feed forward
            public static final double DriveMotorFreeSpeedRps = 5676.0 / 60;
            public static final double WheelDiameterMeters = 0.0762;
            public static final double WheelCircumferenceMeters = WheelDiameterMeters * Math.PI;
            // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
            public static final double DriveMotorReduction = (45.0 * 22) / (DriveMotorPinionTeeth * 15);
            public static final double DriveWheelFreeSpeedRps = (DriveMotorFreeSpeedRps * WheelCircumferenceMeters)
                    / DriveMotorReduction;
            public static final double DriveFF = 1 / DriveWheelFreeSpeedRps;
            public static final double DriveEncoderPositionFactor = (WheelDiameterMeters * Math.PI)
                    / DriveMotorReduction; // meters
            public static final double DriveEncoderVelocityFactor = ((WheelDiameterMeters * Math.PI)
                    / DriveMotorReduction) / 60.0; // meters per second
            public static final double TurnEncoderPositionFactor = (2 * Math.PI); // radians
            public static final double TurnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second
        }
    }

    public static final class TestSubsystem {
        public static final int CANID = 1;
        public static final double P = 0.0001;
        public static final double I = 0;
        public static final double D = 1;
        public static final double FF = 0;
        public static final double I_ZONE = 0;
    }

    public static final class Operation {
        public static final boolean FIELD_ORIENTED = true;
    }

    public static final class Robot {
        public static final Measure<Distance> ROBOT_WIDTH = Units.Inches.of(21);
        public static final Measure<Distance> ROBOT_LENGTH = Units.Inches.of(21);
    }
}
