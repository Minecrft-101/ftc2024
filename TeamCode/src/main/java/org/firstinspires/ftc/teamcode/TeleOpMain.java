package org.firstinspires.ftc.teamcode;


import java.lang.Math;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.SubSys.Arm;
import org.firstinspires.ftc.teamcode.SubSys.Stretch;
import org.firstinspires.ftc.teamcode.SubSys.Drive;

import org.firstinspires.ftc.teamcode.Variables.ClawV;
import org.firstinspires.ftc.teamcode.Variables.LimbV;
import org.firstinspires.ftc.teamcode.Variables.DriveV;

@TeleOp(name = "TeleOpMain", group = "_JVBot") // YOU DID NOT HAVE THIS WRITTEN. THIS IS WHAT TELLS THE ROBOT THIS IS AN OPMODE. IMPORT WAS ALSO NOT THERE. -Nick
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        TouchSensor button = hardwareMap.get(TouchSensor.class, "extLimit");
        int val = 0;

        Arm arm = new Arm(hardwareMap.get(DcMotorEx.class,"armRot"), hardwareMap.get(TouchSensor.class, "armLimit"));
        Stretch ext = new Stretch(hardwareMap.get(DcMotorEx.class, "armExt"), hardwareMap.get(TouchSensor.class, "extLimit"));
        Drive drive = new Drive(hardwareMap.get(DcMotorEx.class, "frontLeft"),hardwareMap.get(DcMotorEx.class, "frontRight"),hardwareMap.get(DcMotorEx.class, "backLeft"),hardwareMap.get(DcMotorEx.class, "backRight"), hardwareMap.get(IMU.class, "imu"));
        //Airplane airplane = new Airplane(hardwareMap.get(Servo.class, "yeet"));
        org.firstinspires.ftc.teamcode.SubSys.Claw claw = new org.firstinspires.ftc.teamcode.SubSys.Claw(hardwareMap.get(Servo.class, "handTop"), hardwareMap.get(Servo.class, "handBottom"), hardwareMap.get(Servo.class, "wristLeft"), hardwareMap.get(Servo.class, "wristRight"));
        //nerd

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double left_x = gamepad1.left_stick_x;
                double left_y = -gamepad1.left_stick_y;
                double right_x = gamepad1.right_stick_x;
                if (Math.abs(left_y) < .1) {
                    left_y = 0;
                }
                if (Math.abs(left_x) < .1) {
                    left_x = 0;
                }
                if (Math.abs(right_x) < .1) {
                    right_x = 0;
                }

                if (gamepad1.a) {
                    ext.setPosition(LimbV.stretch_dropOffPos);
                }
                if (gamepad1.b) {
                    ext.setPosition(LimbV.stretch_pickUp);
                }

                if (gamepad1.right_bumper) {
                    arm.setPosition(LimbV.arm_dropOffPos);
                }
                if (gamepad1.left_bumper){
                    arm.setPosition(LimbV.arm_pickUp);
                }

                if (gamepad1.dpad_down) {
                    claw.setTopHand(ClawV.Thand_grab);
                }
                if (gamepad1.dpad_up){
                    claw.setTopHand(ClawV.Thand_drop);
                }

                if (gamepad1.dpad_left) {
                    claw.setTopHand(ClawV.Thand_grab);
                }
                if (gamepad1.dpad_right){
                    claw.setTopHand(ClawV.Thand_drop);
                }

                if (gamepad1.y){
                    claw.setWristPos(ClawV.wrist_pickUpPos);
                }
                if (gamepad1.x){
                    claw.setWristPos(ClawV.wrist_dropOffPos);
                }

                if (gamepad1.right_bumper){
                //    airplane.setPosition(LimbV.airplane_servoPos);
                }

                if (gamepad1.start) {
                    ext.resetStretch();
                    arm.resetShoulder();
                }

                double correctionArm = arm.update();
                double correctionExt = ext.update();

                drive.drive(left_x, left_y, -right_x);

                telemetry.addData("Arm Rotation", arm.getEncoderValue());
                telemetry.addData("Arm Target", arm.getTarget());
                telemetry.addData("arm correction:",correctionArm);
                telemetry.addData("Arm Extension", ext.getEncoderValue());
                telemetry.addData("Arm Extent Target", ext.getTarget());
                telemetry.addData("arm extent correction:",correctionExt);
                telemetry.addData("x_distance",drive.getXDistance());
                telemetry.addData("y_distance",drive.getYDistance());
                telemetry.update();
            }
        }
    }
}