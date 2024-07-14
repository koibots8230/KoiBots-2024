// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.devices.GuliKit;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Test;


public class RobotContainer {
    private final GuliKit controller;
    private final Test testSubsystem;
    private final Drivetrain drivetrain;

    public RobotContainer(boolean isReal) {
        controller = new GuliKit(0);
        testSubsystem = new Test(isReal);
        drivetrain = new Drivetrain(isReal);
        configureBindings();
    }

    private void configureBindings() {
        testSubsystem.setDefaultCommand(new InstantCommand(() ->
                testSubsystem.setVelocity(Units.RPM.of(controller.getRightJSY() * 1000)),
                testSubsystem));

        drivetrain.setDefaultCommand(new InstantCommand(() ->
                drivetrain.drive(controller.getLeftJSX()*3, controller.getLeftJSY()*3, controller.getRightJSX()*4),
                drivetrain));

        controller.y.onTrue(new InstantCommand(() ->
                drivetrain.setModule(new SwerveModuleState(
                        2, new Rotation2d()), Drivetrain.SwerveModules.frontLeft),
                drivetrain));
        controller.x.onTrue(new InstantCommand(() ->
                drivetrain.setModule(new SwerveModuleState(
                        2, new Rotation2d()), Drivetrain.SwerveModules.frontRight),
                drivetrain));
        controller.b.onTrue(new InstantCommand(() ->
                drivetrain.setModule(new SwerveModuleState(
                        2, new Rotation2d()), Drivetrain.SwerveModules.backLeft),
                drivetrain));
        controller.a.onTrue(new InstantCommand(() ->
                drivetrain.setModule(new SwerveModuleState(
                        2, new Rotation2d()), Drivetrain.SwerveModules.backRight),
                drivetrain));

        controller.y.onFalse(new InstantCommand(() ->
                drivetrain.setModule(new SwerveModuleState(
                        0, new Rotation2d()), Drivetrain.SwerveModules.frontLeft),
                drivetrain));
        controller.x.onFalse(new InstantCommand(() ->
                drivetrain.setModule(new SwerveModuleState(
                        0, new Rotation2d()), Drivetrain.SwerveModules.frontRight),
                drivetrain));
        controller.b.onFalse(new InstantCommand(() ->
                drivetrain.setModule(new SwerveModuleState(
                        0, new Rotation2d()), Drivetrain.SwerveModules.backLeft),
                drivetrain));
        controller.a.onFalse(new InstantCommand(() ->
                drivetrain.setModule(new SwerveModuleState(
                        0, new Rotation2d()), Drivetrain.SwerveModules.backRight),
                drivetrain));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    public void debug() {
        controller.updateDebug();
    }
}
