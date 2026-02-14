
package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Polar Drivetrain go brrr")
public class Drivetrain_Polar  extends OpMode {

    public TelemetryManager panelsTelemetry;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    boolean Robot = true;

    double front;
    double side;
    double turn;

    GoBildaPinpointDriver imu;

    @Override
    public void init() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(GoBildaPinpointDriver.class, "imu");

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper to drive in robot relative");
        telemetry.addLine("The left joystick sets the robot direction");
        telemetry.addLine("Moving the right joystick left and right turns the robot");

        panelsTelemetry.debug("X", imu.getPosX(DistanceUnit.INCH));
        panelsTelemetry.debug("Y", imu.getPosY(DistanceUnit.INCH));
        panelsTelemetry.debug("Heading", imu.getHeading(AngleUnit.DEGREES));
        panelsTelemetry.addData("Gamepad 1 X", gamepad1.left_stick_x);
        panelsTelemetry.addData("Gamepad 1 Y", gamepad1.left_stick_y);
        panelsTelemetry.addData("Gamepad 1 Direction", gamepad1.right_stick_x);

        panelsTelemetry.update(telemetry);

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.circle) {
            imu.resetPosAndIMU();
        }

        if (Robot) {
            front = Caertesian_Polar(-gamepad1.left_stick_y, gamepad1.left_stick_x)[0];
            side = Caertesian_Polar(-gamepad1.left_stick_y, gamepad1.left_stick_x)[1];
            turn = gamepad1.right_stick_x;
            drive(front, side, turn);
        } else {
            front = Caertesian_Polar(-gamepad1.left_stick_y, gamepad1.left_stick_x)[0];

            // TODO: Check if the imu records 0-360 or -180 - 180
            side = Caertesian_Polar(-gamepad1.left_stick_y, gamepad1.left_stick_x)[1] - (2*Math.PI - imu.getHeading(AngleUnit.RADIANS));

            turn = gamepad1.right_stick_x;
            drive(front, side, turn);
        }

        if (gamepad1.left_bumper){
            Robot = true;
        } else if(gamepad1.right_bumper)  {
            Robot = false;
        }

        if (gamepad1.triangle){
            frontLeftDrive.setPower(1);
            frontRightDrive.setPower(1);
            backLeftDrive.setPower(1);
            backRightDrive.setPower(1);
        }

        if (gamepad1.square){
            frontLeftDrive.setPower(-1);
            frontRightDrive.setPower(-1);
            backLeftDrive.setPower(-1);
            backRightDrive.setPower(-1);
        }

        imu.update();
    }

    public double[] Caertesian_Polar(double forward, double right){
        double magnitude = Math.hypot(forward, right);
        double theta = Math.atan2(forward, right);
        double[] result = {magnitude, theta};

        return result;
    }


    public void drive(double power, double theta, double rotate) {

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(cos), Math.abs(sin));

        double frontLeftPower = power * cos/max + rotate;
        double frontRightPower = power * cos/max - rotate;
        double backRightPower = power * cos/max - rotate;
        double backLeftPower = power * cos/max + rotate;

        if ((power + Math.abs(rotate)) > 1){
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
}
