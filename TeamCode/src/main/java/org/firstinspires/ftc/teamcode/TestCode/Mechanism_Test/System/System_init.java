package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class System_init {

   public DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
   public DcMotorEx intake, transfer;
   public DcMotorEx shooter;
   public Servo Hood, gateLeft, gateRight;
   public GoBildaPinpointDriver pinpoint;
   public Limelight3A limelight3A;

    public void init (HardwareMap Hwmap){
        frontLeftDrive = Hwmap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = Hwmap.get(DcMotorEx.class, "front_right_drive");
        backLeftDrive = Hwmap.get(DcMotorEx.class, "back_left_drive");
        backRightDrive = Hwmap.get(DcMotorEx.class, "back_right_drive");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);                      //TODO: CHECK IF ANY OTHER MOTORS NEED TO BE REVERSED
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pinpoint = Hwmap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.setOffsets(0,0, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.recalibrateIMU();
        pinpoint.resetPosAndIMU();
        pinpoint.setHeading(Math.PI/2, AngleUnit.RADIANS);

        intake = Hwmap.get(DcMotorEx.class, "intake");
        shooter = Hwmap.get(DcMotorEx.class, "shooter");
        transfer = Hwmap.get(DcMotorEx.class, "Transfer");

        Hood = Hwmap.get(Servo.class, "Hood");
        gateLeft = Hwmap.get(Servo.class, "GateL");
        gateRight = Hwmap.get(Servo.class, "GateR");
        gateRight.setDirection(Servo.Direction.REVERSE);

        limelight3A = Hwmap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
    }
}