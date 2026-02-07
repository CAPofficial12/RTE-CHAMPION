
package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Rene Drivetrain go brrr")
public class Drivetrain_Caertesian  extends OpMode {

    public TelemetryManager panelsTelemetry;
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    boolean Robot = true;

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
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
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


    public void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                imu.getHeading(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {

        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxSpeed = 1.0;
        double maxPower;

        maxPower = Math.max(Math.abs(frontRightPower), Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));


        frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
