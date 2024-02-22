package org.firstinspires.ftc.teamcode.Variables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class LimbV {
    //arm constants
    public static double arm_GEAR_RATIO = (54.0/10.0)*5*5*5;
    public static double arm_kP = 0.0005;
    public static double arm_kI = 0;
    public static double arm_kD = 0;
    public static double arm_kG = 0;
    public static int arm_maxVel = 5000;
    public static int arm_maxAccel = 5000;
    public static int arm_maxJerk = 5000;
    public static int arm_dropOffPos = 2200;
    public static int arm_pickUp = 0;
    public static int arm_stow = 0;

    //Stretch Constants
    public static double stretch_GEAR_RATIO = 5*5;
    public static double stretch_kP = 0.013;
    public static double stretch_kI = 0;
    public static double stretch_kD = 0;
    public static double stretch_kG = 0;
    public static int stretch_maxVel = 5000;
    public static int stretch_maxAccel = 5000;
    public static int stretch_maxJerk = 5000;
    public static int stretch_dropOffPos = -2900;
    public static int stretch_pickUp = -2400;
    public static int stretch_stow = 0;

    // Airplane Launcher
    public static double airplane_servoPos = 1;


}