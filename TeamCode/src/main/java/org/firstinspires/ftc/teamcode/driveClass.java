package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "driveClass", group = "TeleOp")

public class driveClass extends LinearOpMode {

    DcMotor rightMotor;
    DcMotor leftMotor;
    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize Motors
        rightMotor = hardwareMap.dcMotor.get("test_motor");
        leftMotor = hardwareMap.dcMotor.get("left_motor");

        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);

        //Define Drive Variables
        double x;
        double y;
        double right;
        double left;

        waitForStart();
        resetStartTime();

        while(opModeIsActive()) {
            x = gamepad1.left_stick_x;
            y = -gamepad1.left_stick_y;

            left = y+x;
            right = y-x;

            leftMotor.setPower(left);
            rightMotor.setPower(right);

            telemetry.addData("  Text", "***Program Information***");
            telemetry.addData("  Runtime", getRuntime() + " seconds");
            telemetry.addData("  Left Motor Turn", leftMotor.getCurrentPosition() + " Encoder Counts");
            telemetry.addData("  Right Motor Turn", rightMotor.getCurrentPosition() + " Encoder Counts");
            telemetry.update();
        }
    }
}
