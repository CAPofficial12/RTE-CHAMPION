
package org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.Drivetrain;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.TestCode.Mechanism_Test.System.System_init;

@TeleOp(name = "Rene Drivetrain go brrr")
public class Drivetrain_Caertesian  extends OpMode {

    public TelemetryManager panelsTelemetry;
    System_init system_init = new System_init();
    boolean Robot = true;


    @Override
    public void init() {
        system_init.init(hardwareMap);

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

        panelsTelemetry.debug("X", system_init.pinpoint.getPosX(DistanceUnit.INCH));
        panelsTelemetry.debug("Y", system_init.pinpoint.getPosY(DistanceUnit.INCH));
        panelsTelemetry.debug("Heading", system_init.pinpoint.getHeading(AngleUnit.DEGREES));
        panelsTelemetry.addData("Gamepad 1 X", gamepad1.left_stick_x);
        panelsTelemetry.addData("Gamepad 1 Y", gamepad1.left_stick_y);
        panelsTelemetry.addData("Gamepad 1 Direction", gamepad1.right_stick_x);

        panelsTelemetry.update(telemetry);

        // If you press the A button, then you reset the Yaw to be zero from the way
        // the robot is currently pointing
        if (gamepad1.circle) {
            system_init.pinpoint.resetPosAndIMU();
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
            system_init.frontLeftDrive.setPower(1);
            system_init.frontRightDrive.setPower(1);
            system_init.backLeftDrive.setPower(1);
            system_init.backRightDrive.setPower(1);
        }

        if (gamepad1.square){
            system_init.frontLeftDrive.setPower(-1);
            system_init.frontRightDrive.setPower(-1);
            system_init.backLeftDrive.setPower(-1);
            system_init.backRightDrive.setPower(-1);
        }

        system_init.pinpoint.update();
    }


    public void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        theta = AngleUnit.normalizeRadians(theta -
                system_init.pinpoint.getHeading(AngleUnit.RADIANS));

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


        system_init.frontLeftDrive.setPower(maxSpeed * (frontLeftPower / maxPower));
        system_init.frontRightDrive.setPower(maxSpeed * (frontRightPower / maxPower));
        system_init.backLeftDrive.setPower(maxSpeed * (backLeftPower / maxPower));
        system_init.backRightDrive.setPower(maxSpeed * (backRightPower / maxPower));
    }
}
