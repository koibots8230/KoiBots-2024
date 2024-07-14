// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Test;


public class RobotContainer {
    private final GenericHID controller;
    private final Test testSubsystem;
    private final Drivetrain drivetrain;

    public RobotContainer(boolean isReal) {
        controller = new GenericHID(0);
        testSubsystem = new Test(isReal);
        drivetrain = new Drivetrain(isReal);
        configureBindings();
    }

    private void configureBindings() {
        testSubsystem.setDefaultCommand(new InstantCommand(() ->
                testSubsystem.setVelocity(Units.RPM.of(controller.getRawAxis(1) * 1000)),
                testSubsystem));
        drivetrain.setDefaultCommand(new InstantCommand(() ->
                drivetrain.drive(controller.getRawAxis(0), controller.getRawAxis(1), controller.getRawAxis(2)),
                drivetrain));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
