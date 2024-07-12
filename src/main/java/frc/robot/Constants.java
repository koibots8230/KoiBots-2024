package frc.robot;

import edu.wpi.first.units.*;

public final class Constants {
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
            public static final double I_ZONE = 0;
            public static final double GEARING = 5.08;
            public static final Measure<Distance> WHEEL_DIAMETER = Units.Inches.of(3.0);
        }

        public static final class Turn {
            public static final int ID_FRONT_LEFT = 2;
            public static final int ID_FRONT_RIGHT = 4;
            public static final int ID_BACK_LEFT = 6;
            public static final int ID_BACK_RIGHT = 8;
            public static final double P = 0.0001;
            public static final double I = 0;
            public static final double D = 1;
            public static final double S = 0;
        }
    }

    public static final class Operation {
        public static final boolean FIELD_ORIENTED = true;
    }

    public static final class Robot {
        public static final Measure<Distance> ROBOT_WIDTH = Units.Inches.of(21);
        public static final Measure<Distance> ROBOT_LENGTH = Units.Inches.of(21);
    }
}
