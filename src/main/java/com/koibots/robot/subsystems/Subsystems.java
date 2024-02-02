// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot.subsystems;

import com.koibots.robot.subsystems.intake.IntakePivot.IntakePivot;
import com.koibots.robot.subsystems.swerve.Swerve;
import java.util.function.Supplier;

public class Subsystems {
    private static Swerve swerveInstance;
    public static Supplier<Swerve> Swerve;
    public static IntakePivot intakePivotInstance;
    public static Supplier<IntakePivot> IntakePivot;

    static {
        Swerve =
                () -> {
                    swerveInstance = new Swerve();
                    Swerve =
                            () -> {
                                return swerveInstance;
                            };

                    return swerveInstance;
                };

        IntakePivot =
                () -> {
                    // intakePivotInstance = new IntakePivot();
                    // IntakePivot = () -> {
                    //     return intakePivotInstance;
                    // };

                    return intakePivotInstance;
                };
    }
}
