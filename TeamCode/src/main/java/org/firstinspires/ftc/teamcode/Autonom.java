/*

package org.firstinspires.ftc.teamcode;


import java.util.concurrent.atomic.AtomicLong;

import com.acmerobotics.dashboard.FtcDashboard;
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

@Autonomous(name = "Autonom", group = "_JVBot") // YOU DID NOT HAVE THIS WRITTEN. THIS IS WHAT TELLS THE ROBOT THIS IS AN OPMODE. IMPORT WAS ALSO NOT THERE. -Nick
public class Autonom extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        TouchSensor button = hardwareMap.get(TouchSensor.class, "extLimit");

        Arm arm = new Arm(hardwareMap.get(DcMotorEx.class,"armRot"), hardwareMap.get(TouchSensor.class, "armLimit"));
        Stretch ext = new Stretch(hardwareMap.get(DcMotorEx.class, "armExt"), hardwareMap.get(TouchSensor.class, "extLimit"));
        Drive drive = new Drive(hardwareMap.get(DcMotorEx.class, "frontLeft"),hardwareMap.get(DcMotorEx.class, "frontRight"),hardwareMap.get(DcMotorEx.class, "backLeft"),hardwareMap.get(DcMotorEx.class, "backRight"), hardwareMap.get(IMU.class, "imu"));
        //Airplane airplane = new Airplane(hardwareMap.get(Servo.class, "yeet"));
        Claw claw = new Claw(hardwareMap.get(Servo.class, "wrist"),hardwareMap.get(Servo.class, "topHand"),hardwareMap.get(Servo.class, "bottomHand"));
        //nerd

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                drive.setTarget(0,0 );
                while(drive.getxTarget() != drive.getXDistance() && drive.getyTarget() != drive.getYDistance()){ drive.update(); }

                drive.setTarget(500,500);
                while(drive.getxTarget() != drive.getXDistance() && drive.getyTarget() != drive.getYDistance()){ drive.update(); }

                ext.setPosition(RobotConstants.stretch_dropOffPos);
                arm.setPosition(RobotConstants.arm_dropOffPos);
                claw.set_wristPosition(RobotConstants.wrist_dropOffPos);
                while(Arm.armTarget() != Arm.getPosition());
                claw.set_topHandPosition(RobotConstants.topHand_dropOffPos);
                claw.set_bottomHandPosition(RobotConstants.bottomHand_dropOffPos);
                sleep(1000);

                ext.setPosition(RobotConstants.stretch_pickUp);
                arm.setPosition(RobotConstants.arm_pickUp);
                claw.set_wristPosition(RobotConstants.wrist_pickUpPos);
                claw.set_topHandPosition(RobotConstants.topHand_pickUpPos);
                claw.set_bottomHandPosition(RobotConstants.bottomHand_pickUpPos);
                sleep(100);

                drive.setTarget(500,500);
                while(drive.getxTarget() != drive.getxTarget() && drive.getyTarget() != drive.getyTarget()){}

                drive.setTarget(1000,1000);
                while(drive.getxTarget() != drive.getxTarget() && drive.getyTarget() != drive.getyTarget()){}









                //double correctionArm = arm.update();
                //double correctionExt = ext.update();

                //telemetry.addData("Arm Rotation", arm.getEncoderValue());
                //telemetry.addData("Arm Target", arm.getTarget());
                //telemetry.addData("arm correction:",correctionArm);
                //telemetry.addData("Arm Extension", ext.getEncoderValue());
                //telemetry.addData("Arm Extent Target", ext.getTarget());
                //telemetry.addData("arm extent correction:",correctionExt);
                telemetry.addData("x_distance",drive.getXDistance());
                telemetry.addData("y_distance",drive.getYDistance());
                telemetry.update();
            }
        }
    }
}
*/