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
    public static int stretch_pickUp = 0;

    //drive constants
    public static final float yawMax = 5;
    public static final float yawCheck = 2.5F;
    public static final double MULTIPLIER = 1;
    public static final double STICK_TOLERANCE = 0.1;
    public static final double MINIMUM_TURNING_SPEED = 0.05;
    public static final double ANGULAR_TOLERANCE = Math.toRadians(0.5);
    public static double x_kP = 0.009;
    public static final double x_kI = 0;
    public static double x_kD = 0.0001;
    public static final double x_kG = 0;
    public static int x_maxVel = 5000;
    public static int x_maxAccel = 5000;
    public static int x_maxJerk = 5000;
    public static double y_kP = 0.006;
    public static final double y_kI = 0;
    public static double y_kD = 0.001;
    public static final double y_kG = 0;
    public static int y_maxVel = 5000;
    public static int y_maxAccel = 5000;
    public static int y_maxJerk = 5000;
    public static int yTarget = 0;
    public static int xTarget = 500;
    public static double drive_fudge = 1;


    // Airplane Launcher
    public static double airplane_servoPos = 1;

    // Claw
    public static double wrist_dropOffPos = 1;
    public static double hand_dropOffPos = 1;
    public static double wrist_pickUpPos = 1;
    public static double hand_pickUpPos = 1;
}