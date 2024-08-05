// Copyright (c) 2024 FRC 8230 - The KoiBots
// https://github.com/koibots8230

package com.koibots.robot;

import static com.koibots.robot.subsystems.Subsystems.*;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private RobotContainer robotContainer;

    List<SendableChooser<Boolean>> modulesEnabled = new ArrayList<SendableChooser<Boolean>>();

    @Override
    public void robotInit() {
        Logger.recordMetadata("RobotName", "Swerve Chassis");
        Logger.addDataReceiver(new WPILOGWriter());

        //CameraServer.startAutomaticCapture();

        if (!DriverStation.isFMSAttached()) {
            Logger.addDataReceiver(new NT4Publisher());
            SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
        }

        if (isReal()) {
            LoggedPowerDistribution.getInstance(0, PowerDistribution.ModuleType.kRev);
        }

        Logger.start();

        robotContainer = new RobotContainer();
        robotContainer.registerAutos();

        SmartDashboard.putData(Intake.get());
        SmartDashboard.putData(Indexer.get());
        SmartDashboard.putData(Shooter.get());
        SmartDashboard.putData("Swerve Subsystem", Swerve.get());

        LEDs.get().send_to_rp2040(1);
        Vision.get();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        if (robotContainer.getAutonomousRoutine() != null) {
            robotContainer.getAutonomousRoutine().schedule();
        }
        LEDs.get().send_to_rp2040(4);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        robotContainer.configureButtonBindings();
        LEDs.get().send_to_rp2040(4);
    }

    @Override
    public void disabledInit() {
        Swerve.get().stop();
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {
        robotContainer.configureTestBinds();
    }
}
