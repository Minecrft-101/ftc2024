package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;

@TeleOp(name = "ServoCalibrate", group = "_JVBot")
public class servoWristCheck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo c = hardwareMap.get(Servo.class, "wristRight");
        Servo d = hardwareMap.get(Servo.class, "wristLeft");


        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                c.setPosition(RobotConstants.wrist_dropOffPos);
                d.setPosition(RobotConstants.wrist_dropOffPos);
                telemetry.addData("Right servo pos", c.getPosition());
                telemetry.addData("Left servo pos", c.getPosition());
                telemetry.update();
            }
        }
    }
}