

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

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (stage == 0){
                    //stage 1
                    drive.setTarget(-650, 540);
                }

                if (stage == 1) {
                    //stage 2
                    arm.setPosition(LimbV.arm_dropOffPos);
                    ext.setPosition(LimbV.stretch_dropOffPos);
                }
                if (stage == 2) {
                    //stage 3
                    claw.setWristPos(ClawV.wrist_dropOffPos);
                    sleep(100);
                    claw.setBottomHand(ClawV.Bhand_drop);
                    sleep(100);
                    claw.setTopHand(ClawV.Thand_drop);
                }
                if (drive.isAtTarget(5) && drive.getxTarget() != -650 && drive.getyTarget() != 540) {
                    stage = 1;
                    sleep(100);

                }
                if (arm.isAtTarget(5) && arm.getTarget() == LimbV.arm_dropOffPos && ext.isAtTarget(50) && ext.getTarget() == LimbV.stretch_dropOffPos) {
                    stage = 2;
                    sleep(100);

                }
                if ((stage == 2) && (claw.TopHandPos() == ClawV.Thand_drop)) {
                    stage = 3;
                    sleep(100);

                }

                arm.update();
                ext.update();
                drive.update();
            }

        }
    }
}
