
package org.firstinspires.ftc.teamcode;

import java.lang.Math;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.SubSys.Arm;
import org.firstinspires.ftc.teamcode.SubSys.Stretch;

@TeleOp(name = "TeleOpMain", group = "_JVBot") // YOU DID NOT HAVE THIS WRITTEN. THIS IS WHAT TELLS THE ROBOT THIS IS AN OPMODE. IMPORT WAS ALSO NOT THERE. -Nick
public class TeleOpMain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        Arm arm = new Arm(hardwareMap.get(DcMotorEx.class,"armRot"), hardwareMap.get(TouchSensor.class, "armLimit"));
        Stretch ext = new Stretch(hardwareMap.get(DcMotorEx.class, "armExt"), hardwareMap.get(TouchSensor.class, "extLimit"));
        //arm.setPosition(RobotConstants.arm_pickUp);

        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double left_y = gamepad1.left_stick_y;
                if (Math.abs(left_y)<.1) {
                    left_y = 0;
                }


                if (gamepad1.a) {
                    arm.setPosition(RobotConstants.arm_dropOffPos);
                }
                if (gamepad1.b){
                    arm.setPosition(RobotConstants.arm_pickUp);
                }

                if (gamepad1.y){
                    arm.resetShoulder();
                }

                double correction = arm.update();


                ext.moveManual(left_y);

                telemetry.addData("Arm Rotation", arm.getEncoderValue());
                telemetry.addData("Arm Target", arm.getTarget());
                telemetry.addData("arm correction:",correction);
                telemetry.update();
            }
        }
    }
}