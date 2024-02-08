package org.firstinspires.ftc.teamcode.SubSys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public class Claw {

    private Servo w;
    private Servo h;

    public Claw(Servo hand, Servo wrist) {
        this.w = wrist;
        this.h = hand;
    }
    public void set_handPosition(double handPos){
        this.h.setPosition(handPos);
    }

    public void set_wristPosition(double wristPos) {
        this.w.setPosition(wristPos);
    }
}
