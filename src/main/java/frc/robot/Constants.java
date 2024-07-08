package frc.robot;

import edu.wpi.first.units.*;
import frc.robot.devices.MotorDefinition;
import static frc.robot.devices.MotorDefinition.MotorType.*;

public final class Constants {
    public static class Motors {
        /* NON-DRIVETRAIN */
        public static MotorDefinition Intake = new MotorDefinition(
                12, 0.0001, 0, 1, 0, 0, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition Indexer = new MotorDefinition(
                13, 0.0001, 0, 1, 0, 0, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition ShooterTop = new MotorDefinition(
                14, 0.0001, 0, 1, 0, 0, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition ShooterBottom = new MotorDefinition(
                15, 0.0001, 0, 1, 0, 0, CANSPARKMAX_BRUSHLESS);

        /* DRIVETRAIN */
        public static MotorDefinition FrontLeftDrive = new MotorDefinition(
                1, 0.0001, 0, 1, 0, 0, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition FrontLeftTurn = new MotorDefinition(
                2, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition FrontRightDrive = new MotorDefinition(
                3, 0.0001, 0, 1, 0, 0, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition FrontRightTurn = new MotorDefinition(
                4, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition BackLeftDrive = new MotorDefinition(
                5, 0.0001, 0, 1, 0, 0, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition BackLeftTurn = new MotorDefinition(
                6, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition BackRightDrive = new MotorDefinition(
                7, 0.0001, 0, 1, 0, 0, CANSPARKMAX_BRUSHLESS);
        public static MotorDefinition BackRightTurn = new MotorDefinition(
                8, CANSPARKMAX_BRUSHLESS);
    }

    public static class Drivetrain {
        public static class Turning {
            public static final double P = 0.0001;
            public static final double I = 0;
            public static final double D = 1;
            public static final double S = 0;
            public static final double V = 0;
            public static final double A = 0;
        }
        public static class Driving {
            public static final double P = 0.0001;
            public static final double I = 0;
            public static final double D = 1;
            public static final double S = 0;
            public static final double V = 0;
            public static final double A = 0;
            public static final double GEARING = 5.08;
            public static final Measure<Distance> WHEEL_DIAMETER = Units.Inches.of(3.0);
        }
    }

    public static class Operation {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final boolean FIELD_ORIENTED = true;
    }
}
