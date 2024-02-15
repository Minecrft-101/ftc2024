package org.firstinspires.ftc.teamcode.SubSys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
public class Claw {

    private Servo wL;
    private Servo wR;
    private Servo hT;
    private Servo hB;

    public Claw(Servo handTop, Servo handBottom, Servo wristLeft, Servo wristRight) {
        this.wL = wristLeft;
        this.wR = wristRight;
        this.hT = handTop;
        this.hB = handBottom;
        this.wR.setDirection(Servo.Direction.REVERSE);
    }
    public void setTopHand(double handPos){
        this.hT.setPosition(handPos);
    }
    public void setBottomHand(double handPos){
        this.hB.setPosition(handPos);
    }

    public void setWristPos(double wristPos) {
        this.wL.setPosition(wristPos);
        this.wR.setPosition(wristPos);
    }
}
