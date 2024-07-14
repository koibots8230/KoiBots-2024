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
            public static final CANSparkBase.IdleMode IDLE_MODE = CANSparkBase.IdleMode.kBrake;
            public static final Measure<Current> CURRENT_LIMIT = Units.Amps.of(50);
            public static final int PINION_TEETH = 14;
            public static final double REDUCTION = (45.0 * 22) / (PINION_TEETH * 15);
            public static final Measure<Velocity<Angle>> MOTOR_FREE_SPEED =
                    Units.RotationsPerSecond.of(5676.0 / 60);
            public static final Measure<Distance> WHEEL_DIAMERER = Units.Meters.of(0.0762);
            public static final Measure<Distance> WHEEL_CIRCUMFERENCE = WHEEL_DIAMERER.times(Math.PI);
            public static final Measure<Velocity<Distance>> WHEEL_FREE_SPEED =
                    Units.MetersPerSecond.of((MOTOR_FREE_SPEED.in(Units.RPM) *
                            WHEEL_CIRCUMFERENCE.in(Units.Meters)) / REDUCTION);
            public static final double ENCODER_POSITION_FACTOR = (WHEEL_DIAMERER.in(Units.Meters) * Math.PI)
                    / REDUCTION; // meters
            public static final double ENCODER_VELOCITY_FACTOR = ((WHEEL_DIAMERER.in(Units.Meters) * Math.PI)
                    / REDUCTION) / 60.0; // meters per second

            public static final int ID_FRONT_LEFT = 1;
            public static final int ID_FRONT_RIGHT = 3;
            public static final int ID_BACK_LEFT = 5;
            public static final int ID_BACK_RIGHT = 7;
            public static final double P = 0.0001;
            public static final double I = 0;
            public static final double D = 1;
            public static final double FF = 1 / WHEEL_FREE_SPEED.in(Units.MetersPerSecond);
            public static final double MIN_OUT = -1;
            public static final double MAX_OUT = 1;
        }

        public static final class Turn {
            public static final CANSparkBase.IdleMode IDLE_MODE = CANSparkBase.IdleMode.kBrake;
            public static final Measure<Current> CURRENT_LIMIT = Units.Amps.of(20);
            public static final boolean INVERT = true;
            public static final double ENCODER_POSITION_FACTOR = (2 * Math.PI); // radians
            public static final double ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

            public static final int ID_FRONT_LEFT = 2;
            public static final int ID_FRONT_RIGHT = 4;
            public static final int ID_BACK_LEFT = 6;
            public static final int ID_BACK_RIGHT = 8;
            public static final double P = 0.0001;
            public static final double I = 0;
            public static final double D = 1;
            public static final double FF = 0;
            public static final double MIN_IN = 0;
            public static final double MAX_IN = ENCODER_POSITION_FACTOR;
            public static final double MIN_OUT = -1;
            public static final double MAX_OUT = 1;
        }
    }

    public static final class TestSubsystem {
        public static final int CANID = 99;
        public static final double P = 0.0001;
        public static final double I = 0;
        public static final double D = 1;
        public static final double FF = 0;
        public static final double I_ZONE = 0;
    }

    public static final class Operation {
        public static final boolean FIELD_ORIENTED = true;
        public static final double DEADBAND = 0.015;
        public static final boolean DEADBAND_CENTER = true; //only apply deadband if both axis are near zero
    }

    public static final class Robot {
        public static final Measure<Distance> ROBOT_WIDTH = Units.Inches.of(21);
        public static final Measure<Distance> ROBOT_LENGTH = Units.Inches.of(21);
    }
}
