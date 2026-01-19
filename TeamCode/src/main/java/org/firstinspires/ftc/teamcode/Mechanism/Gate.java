package org.firstinspires.ftc.teamcode.Mechanism;

import com.qualcomm.robotcore.hardware.Servo;

public class Gate {
    Servo GateRight;
    Servo GateLeft;

    public void open(){
        GateLeft.setPosition(0.5);
        GateRight.setPosition(0.5);
    }

    public void close(){
        GateLeft.setPosition(0.7);
        GateRight.setPosition(0.7);
    }

}
