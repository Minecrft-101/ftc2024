

package org.firstinspires.ftc.teamcode;


import java.util.concurrent.atomic.AtomicLong;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.SubSys.Arm;
import org.firstinspires.ftc.teamcode.SubSys.Claw;
import org.firstinspires.ftc.teamcode.SubSys.Drive;
import org.firstinspires.ftc.teamcode.SubSys.Stretch;
import org.firstinspires.ftc.teamcode.Variables.ClawV;
import org.firstinspires.ftc.teamcode.Variables.LimbV;

@Autonomous(name = "BAutonomL", group = "_JVBot")
public class BAutonomL extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        TouchSensor button = hardwareMap.get(TouchSensor.class, "extLimit");
        int stage = 0;
        Arm arm = new Arm(hardwareMap.get(DcMotorEx.class,"armRot"), hardwareMap.get(TouchSensor.class, "armLimit"));
        Stretch ext = new Stretch(hardwareMap.get(DcMotorEx.class, "armExt"), hardwareMap.get(TouchSensor.class, "extLimit"));
        Drive drive = new Drive(hardwareMap.get(DcMotorEx.class, "frontLeft"),hardwareMap.get(DcMotorEx.class, "frontRight"),hardwareMap.get(DcMotorEx.class, "backLeft"),hardwareMap.get(DcMotorEx.class, "backRight"), hardwareMap.get(IMU.class, "imu"));
        //Airplane airplane = new Airplane(hardwareMap.get(Servo.class, "yeet"));
        Claw claw = new org.firstinspires.ftc.teamcode.SubSys.Claw(hardwareMap.get(Servo.class, "handTop"), hardwareMap.get(Servo.class, "handBottom"), hardwareMap.get(Servo.class, "wristLeft"), hardwareMap.get(Servo.class, "wristRight"));
        //nerd

        claw.setWristPos(ClawV.wrist_stow);

        sleep(100);
        claw.setBottomHand(ClawV.Bhand_grab);
        claw.setTopHand(ClawV.Thand_grab);


        drive.yawReset();
        ext.resetStretch();
        arm.resetShoulder();

        boolean doIDrive = true;

        waitForStart();

        while (opModeIsActive()) {
            if (stage == 0 && (drive.getxTarget() != -650 || drive.getyTarget() != 540)){
                //stage 1
                drive.setTarget(-650, 540);
            } else if (stage == 1 && (arm.getTarget() != LimbV.arm_dropOffPos || ext.getTarget() != LimbV.stretch_dropOffPos)) {
                //stage 2
                arm.setPosition(LimbV.arm_dropOffPos);
                ext.setPosition(LimbV.stretch_dropOffPos);
            } else if (stage == 2 && (drive.getxTarget() != -650 || drive.getyTarget() != 980)){
                //stage 3
                drive.setTarget(-650, 980);
            } else if (stage == 3) {
                //stage 4
                claw.setWristPos(ClawV.wrist_dropOffPos);
                sleep(100);
                claw.setBottomHand(ClawV.Bhand_drop);
                sleep(100);
                claw.setTopHand(ClawV.Thand_drop);
            }
            if (stage == 0 && drive.isAtTarget(10) && (drive.getxTarget() == -650) && (drive.getyTarget() == 540)) {
                stage = 1;
                doIDrive = false;
                sleep(100);

            } else if (stage == 1 && arm.isAtTarget(100) && arm.getTarget() == LimbV.arm_dropOffPos && ext.isAtTarget(50) && ext.getTarget() == LimbV.stretch_dropOffPos) {
                stage = 2;
                doIDrive = true;
                sleep(100);

            } else if ((stage == 2) && drive.isAtTarget(5) && (drive.getxTarget() == -650) && (drive.getyTarget() == 980)) {
                stage = 3;
                doIDrive = false;
                sleep(100);

            }

            arm.update();
            ext.update();

            if (!doIDrive) {
                drive.drive(0,0,0);
            } else {
                drive.update();
            }

            telemetry.addData("Stage", stage);
            telemetry.addData("Driving", doIDrive);
            telemetry.addData("X", drive.getXDistance());
            telemetry.addData("Y", drive.getYDistance());
            telemetry.addData("X Target", drive.getxTarget());
            telemetry.addData("Y Target", drive.getyTarget());
            telemetry.addData("X Power", drive.getX_correction());
            telemetry.addData("Y Power", drive.getY_correction());
            telemetry.addData("Rotation", arm.getEncoderValue());
            telemetry.addData("Extension", ext.getEncoderValue());
            telemetry.addData("Rotation Target", arm.getTarget());
            telemetry.addData("Extension Target", ext.getTarget());
            telemetry.addData("Rotation Power", arm.getCorrection());
            telemetry.addData("Extension Power", ext.getCorrection());
            telemetry.update();
        }
    }
}
