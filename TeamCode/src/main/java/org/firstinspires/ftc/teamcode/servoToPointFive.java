package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.SubSys.Arm;
import org.firstinspires.ftc.teamcode.SubSys.Drive;
import org.firstinspires.ftc.teamcode.SubSys.Stretch;

@TeleOp(name = "ServoCalibrate", group = "_JVBot")
public class servoToPointFive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo a = hardwareMap.get(Servo.class, "handTop");
        Servo b = hardwareMap.get(Servo.class, "handBottom");
        Servo c = hardwareMap.get(Servo.class, "wristRight");
        Servo d = hardwareMap.get(Servo.class, "wristLeft");


        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                a.setPosition(0.5);
                b.setPosition(0.5);
                c.setPosition(0.5);
                d.setPosition(0.5);
                telemetry.addData("servo pos",a.getPosition());
                telemetry.update();
            }
        }
    }
}