package org.firstinspires.ftc.teamcode.Variables;

import com.acmerobotics.dashboard.config.Config;

@Config
public class DriveV {
    //Rev motor constants
    public static int TICKS_PER_REV = 28;
    public static int MAX_RPM = 6000;

    //drive constants
    public static float yawMax = 5;
    public static float yawCheck = 2.5F;
    public static double MULTIPLIER = .5;
    public static double STICK_TOLERANCE = 0.1;
    public static double MINIMUM_TURNING_SPEED = 0.2;
    public static double ANGULAR_TOLERANCE = Math.toRadians(.5);
    public static double x_kP = 0.009;
    public static double x_kI = 0;
    public static double x_kD = 0.0001;
    public static double x_kG = 0;
    public static int x_maxVel = 5000;
    public static int x_maxAccel = 5000;
    public static int x_maxJerk = 5000;
    public static double y_kP = 0.006;
    public static double y_kI = 0;
    public static double y_kD = 0.001;
    public static double y_kG = 0;
    public static int y_maxVel = 5000;
    public static int y_maxAccel = 5000;
    public static int y_maxJerk = 5000;
    public static int yTarget = 0;
    public static int xTarget = 500;
    public static double drive_fudge = 1;
}