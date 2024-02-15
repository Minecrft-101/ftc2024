package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.RobotConstants;
import org.firstinspires.ftc.teamcode.SubSys.Claw;

@TeleOp(name = "ServoWristCalibrate", group = "_JVBot")
public class servoWristCheck extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap.get(Servo.class, "handTop"), hardwareMap.get(Servo.class, "handBottom"), hardwareMap.get(Servo.class, "wristLeft"), hardwareMap.get(Servo.class, "wristRight"));

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                claw.setWristPos(RobotConstants.wrist_dropOffPos);
            }
        }
    }
}