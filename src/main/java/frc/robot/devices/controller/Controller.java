package frc.robot.devices.controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public interface Controller {
    GenericHID controller = null;
    Trigger a = null;
    Trigger b = null;
    Trigger x = null;
    Trigger y = null;
    Trigger leftBumper = null;
    Trigger rightBumper = null;
    Trigger leftTrigger = null;
    Trigger rightTrigger = null;
    Trigger start = null;
    Trigger select = null;
    Trigger leftJSPress = null;
    Trigger rightJSPress = null;
    boolean getA();
    boolean getB();
    boolean getX();
    boolean getY();
    boolean getLeftBumper();
    boolean getRightBumper();
    boolean getLeftTrigger();
    boolean getRightTrigger();
    boolean getStart();
    boolean getSelect();
    boolean getLeftJSPress();
    boolean getRightJSPress();
    double getLeftJSXRaw();
    double getLeftJSYRaw();
    double getRightJSXRaw();
    double getRightJSYRaw();
    double getLeftJSX();
    double getLeftJSY();
    double getRightJSX();
    double getRightJSY();
}
