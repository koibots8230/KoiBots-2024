// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.*;
import static edu.wpi.first.units.Units.*;

import com.koibots.lib.controls.EightBitDo;
import com.koibots.lib.util.ShootPosition;
import com.koibots.robot.Constants.*;
import com.koibots.robot.autos.StayPut;
import com.koibots.robot.commands.Intake.IntakeCommand;
import com.koibots.robot.commands.Intake.IntakeShooter;
import com.koibots.robot.commands.Scoring.FeedNote;
import com.koibots.robot.commands.Scoring.Shoot;
import com.koibots.robot.commands.Shooter.SpinUpShooter;
import com.koibots.robot.commands.Swerve.FieldOrientedDrive;
import com.koibots.robot.commands.Swerve.RobotOrientedDrive;
import com.koibots.robot.commands.Swerve.TestDrive;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.List;

public class RobotContainer {
    static EightBitDo driveController = new EightBitDo(0);
    static GenericHID operatorPad = new GenericHID(1);

    List<SendableChooser<Boolean>> modulesEnabled = new ArrayList<>();

    SendableChooser<String> autos = new SendableChooser<>();

    public RobotContainer() {
        registerAutos();

        for (int a = 0; a < 4; a++) {
            SendableChooser<Boolean> module = new SendableChooser<>();

            module.setDefaultOption("Enabled", true);
            module.addOption("Disabled", false);

            SmartDashboard.putData("Module " + a, module);

            modulesEnabled.add(a, module);
        }
    }

    public void registerAutos() {
        NamedCommands.registerCommand(
                "Shoot",
                new Shoot(
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.topSpeed,
                        SetpointConstants.SHOOTER_SPEEDS.SPEAKER.bottomSpeed));
        NamedCommands.registerCommand(
                "Intake", new WaitCommand(0.5));
        NamedCommands.registerCommand(
                "Score_Amp",
                new Shoot(
                        SetpointConstants.SHOOTER_SPEEDS.AMP.topSpeed,
                        SetpointConstants.SHOOTER_SPEEDS.AMP.bottomSpeed));
        NamedCommands.registerCommand("Stay Put", new StayPut());

        PathPlannerLogging.setLogTargetPoseCallback(Swerve.get()::setPathingGoal);

        AutoBuilder.configureHolonomic(
                Swerve.get()::getEstimatedPose,
                Swerve.get()::resetOdometry,
                Swerve.get()::getRelativeSpeeds,
                Swerve.get()::driveRobotRelative,
                ControlConstants.HOLONOMIC_CONFIG,
                () -> false,
                Swerve.get());

        List<String> autoNames = AutoBuilder.getAllAutoNames();

        for (String autoName : autoNames) {
            autos.addOption(autoName, autoName);
        }

        SmartDashboard.putData("Autos", autos);
    }

    public void configureButtonBindings() {
        Swerve.get()
                .setDefaultCommand(
                        new RobotOrientedDrive(
                                () -> -driveController.getLeftY(),
                                () -> -driveController.getLeftX(),
                                () -> -driveController.getRightX(),
                                () -> driveController.getPOV(),
                                () -> driveController.getB()));


        // Trigger intake = new Trigger(() -> driveController.getRightTrigger() > 0.15);
        // intake.onTrue(new IntakeCommand());
        // intake.onFalse(
        //         new ParallelCommandGroup(
        //                 new InstantCommand(() -> Intake.get().setVelocity(RPM.of(0)), Intake.get()),
        //                 new InstantCommand(
        //                         () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()),
        //                 new InstantCommand(() -> RobotContainer.rumbleController(0))));

        Trigger shootAmp = new Trigger(() -> driveController.getLeftTrigger() > 0.15);
        shootAmp.onTrue(new Shoot(SetpointConstants.SHOOTER_SPEEDS.AMP.topSpeed, SetpointConstants.SHOOTER_SPEEDS.AMP.bottomSpeed));
        shootAmp.onFalse(
            new ParallelCommandGroup(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get()),
            new InstantCommand(() -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get())           
        ));

        Trigger intakeShooter = new Trigger(() -> driveController.getRightTrigger() > 0.15);
        intakeShooter.onTrue(
                new ParallelRaceGroup(
                        new StartEndCommand(
                                () ->
                                        Shooter.get()
                                                .setVelocity(
                                                        SetpointConstants.SHOOTER_SPEEDS
                                                                .INTAKE
                                                                .topSpeed,
                                                        SetpointConstants.SHOOTER_SPEEDS
                                                                .INTAKE
                                                                .bottomSpeed),
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get()),
                        new IntakeShooter()));
        intakeShooter.onFalse(
                new ParallelCommandGroup(
                        new InstantCommand(
                                () -> Shooter.get().setVelocity(RPM.of(0), RPM.of(0)),
                                Shooter.get()),
                        new InstantCommand(
                                () -> Indexer.get().setVelocity(RPM.of(0)), Indexer.get()),
                        new InstantCommand(() -> RobotContainer.rumbleController(0))));
    }

    public void configureTestBinds() {
        Swerve.get()
                .setDefaultCommand(
                        new TestDrive(
                                () -> -driveController.getRightX(),
                                () -> -driveController.getRightY(),
                                () -> -driveController.getLeftX(),
                                () -> driveController.getPOV(),
                                () -> driveController.getB(),
                                modulesEnabled));
    }

    public Command getAutonomousRoutine() {
        if (autos.getSelected() != null) {
            Swerve.get().resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile(autos.getSelected()));
            return AutoBuilder.buildAuto(autos.getSelected());
        } else {
            return new InstantCommand();
        }
    }

    public static void rumbleController(double strength) {
        driveController.setRumble(RumbleType.kBothRumble, strength);
    }
}
