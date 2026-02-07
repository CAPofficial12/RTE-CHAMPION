package org.firstinspires.ftc.teamcode.TestCode.Miscallaneous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanism.System_init;

@Autonomous
public class StateIntegerPractice extends OpMode {
    System_init system_init = new System_init();
    enum State {
        WaitForA,
        WaitForB,
        WaitForX,
        Finished
    }
    State state = State.WaitForA;

    @Override
    public void init(){
        system_init.init(hardwareMap);
        state = State.WaitForA;
    }

    @Override
    public void loop(){
        telemetry.addData("Cur State", state);
        switch (state) {
            case WaitForA:
                telemetry.addLine("A: Exit State");
                if (gamepad1.a) {
                    state = State.WaitForB;
                }
                break;
            case WaitForB:
                telemetry.addLine("B: Exit State");
                if (gamepad1.b) {
                    state = State.WaitForX;
                }
                break;
            case WaitForX:
                telemetry.addLine("X: Exit State");
                if (gamepad1.x) {
                    state = State.Finished;
                }
                break;
            default:
                telemetry.addLine("State machine finished");
        }
    }
}
