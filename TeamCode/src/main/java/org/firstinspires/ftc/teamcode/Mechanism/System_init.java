package org.firstinspires.ftc.teamcode.Mechanism;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class System_init {

   DcMotorEx frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;
   DcMotorEx intake;
    public DcMotorEx shooter;
   Servo Hoodleft, Hoodright;
   GoBildaPinpointDriver imu;
   Limelight3A limelight3A;

    public void init (HardwareMap Hwmap){
        frontLeftDrive = Hwmap.get(DcMotorEx.class, "front_left_drive");
        frontRightDrive = Hwmap.get(DcMotorEx.class, "front_right_drive");
        backLeftDrive = Hwmap.get(DcMotorEx.class, "back_left_drive");
        backRightDrive = Hwmap.get(DcMotorEx.class, "back_right_drive");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = Hwmap.get(GoBildaPinpointDriver.class, "imu");

        intake = Hwmap.get(DcMotorEx.class, "intake");
        shooter = Hwmap.get(DcMotorEx.class, "shooter");
        Hoodleft = Hwmap.get(Servo.class, "Hoodleft");
        Hoodright = Hwmap.get(Servo.class, "Hoodright");
        Hoodright.setDirection(Servo.Direction.REVERSE);

        limelight3A = Hwmap.get(Limelight3A.class, "limelight");
        limelight3A.pipelineSwitch(6);
    }
}