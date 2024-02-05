package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class RobotConstants {
    //Rev motor constants
    public static final int TICKS_PER_REV = 28;
    public static final int MAX_RPM = 6000;

    //arm constants
    public static double arm_GEAR_RATIO = (54.0/10.0)*5*5*5;
    public static double arm_kP = 0.001;
    public static double arm_kI = 0;
    public static double arm_kD = 0;
    public static double arm_kG = 0;
    public static int arm_maxVel = 5000;
    public static int arm_maxAccel = 5000;
    public static int arm_maxJerk = 5000;
    public static int arm_dropOffPos = 2000;
    public static int arm_pickUp = 0;

    //Stretch Constants

    public static double stretch_GEAR_RATIO = 5*5;
    public static double stretch_kP = 0.001;
    public static double stretch_kI = 0;
    public static double stretch_kD = 0;
    public static double stretch_kG = 0;
    public static int stretch_maxVel = 5000;
    public static int stretch_maxAccel = 5000;
    public static int stretch_maxJerk = 5000;
    public static int stretch_dropOffPos = -2500;
    public static int stretch_pickUp = -400;
}