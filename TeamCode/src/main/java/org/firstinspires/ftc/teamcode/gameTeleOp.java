package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="Game TeleOp")
public class gameTeleOp extends LinearOpMode {
    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;
    public DcMotor scissor1;
    public DcMotor scissor2;
    public DcMotor pinion;

    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");
        scissor1 = hardwareMap.dcMotor.get("scissor1");
        scissor2 = hardwareMap.dcMotor.get("scissor2");
        pinion = hardwareMap.dcMotor.get("pinion");

        right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        scissor1.setDirection(DcMotorSimple.Direction.REVERSE);
        scissor2.setDirection(DcMotorSimple.Direction.REVERSE);
        pinion.setDirection(DcMotorSimple.Direction.REVERSE);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while(opModeIsActive()) {
            //Drive Code
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
            if(diagX==0 && diagY==0) {
                right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }

            //Arm and Pinion
            double scissor_power = gamepad2.right_stick_y;
            double pinion_power = gamepad2.left_stick_y;

            scissor1.setPower(scissor_power);
            scissor2.setPower(scissor_power);
            pinion.setPower(pinion_power);

        }
    }
    public void setPowers(double right_front_power, double right_back_power, double left_front_power, double left_back_power) {
        right_front.setPower(right_front_power);
        right_back.setPower(right_back_power);
        left_front.setPower(left_front_power);
        left_back.setPower(left_back_power);
    }

}
