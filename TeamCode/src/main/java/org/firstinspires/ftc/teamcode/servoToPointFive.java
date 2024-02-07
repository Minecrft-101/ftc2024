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

@TeleOp(name = "TeleOpMain", group = "_JVBot")
public class servoToPointFive extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo a = hardwareMap.get(Servo.class, "a");

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                a.setPosition(0.5);
                telemetry.addData("servo pos",a.getPosition());
                telemetry.update();
            }
        }
    }
}