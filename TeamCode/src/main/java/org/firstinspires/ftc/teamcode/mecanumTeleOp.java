package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Mecanum TeleOp")
public class mecanumTeleOp extends LinearOpMode {
    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;

    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

        right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {

            double xValue = gamepad1.left_stick_x;
            double yValue = gamepad1.left_stick_y;
            double leftPower =  yValue - xValue;
            double rightPower = yValue + xValue;
            setPowers(rightPower, rightPower, leftPower, leftPower);

            double strafeRight = gamepad1.right_trigger;
            double strafeLeft = gamepad1.left_trigger;

            if(strafeRight > strafeLeft) {
                setPowers(strafeRight, -strafeRight, -strafeRight, strafeRight);
            } else if(strafeLeft > strafeRight) {
                setPowers(-strafeLeft, strafeLeft, strafeLeft, -strafeLeft);
            }
            double diagX = gamepad1.right_stick_x;
            double diagY = gamepad1.right_stick_y;

            telemetry.addData("Thing", diagX );
            telemetry.addData("Thing", diagY);
            telemetry.update();

            if(diagX >0 && diagY <0 ) {
                left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setPowers(0, -0.5,-0.5,0);
            } else if(diagX >0 && diagY >0) {
                left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setPowers(0.5, 0,0,0.5);
            } else if(diagX <0 && diagY <0 ) {
                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setPowers(-0.5, 0,0,-0.5);
            } else if(diagX <0 && diagY >0) {
                left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                setPowers(0, 0.5,0.5,0);
            }

        }
    }
    public void setPowers(double right_front_power, double right_back_power, double left_front_power, double left_back_power) {
        right_front.setPower(right_front_power);
        right_back.setPower(right_back_power);
        left_front.setPower(left_front_power);
        left_back.setPower(left_back_power);
    }

}
