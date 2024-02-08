package org.firstinspires.ftc.teamcode.SubSys;

import com.qualcomm.robotcore.hardware.Servo;

public class Airplane {

    private Servo s;

    public Airplane (Servo launcher) {
        this.s = launcher;
    }
    public void setPosition(double pos){
        this.s.setPosition(pos);
    }

}
