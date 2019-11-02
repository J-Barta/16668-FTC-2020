package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="Game TeleOp")
public class gameTeleOp extends LinearOpMode {
    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;
    public DcMotor scissor1;
    public DcMotor scissor2;
    public DcMotor pinion;

    public Servo claw;
    public Servo foundation1;
    public Servo foundation2;

    public TouchSensor scissor_touch;

    boolean claw_open;
    boolean claw_close;
    double scissor_power;
    double pinion_power;

    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");
        scissor1 = hardwareMap.dcMotor.get("scissor1");
        scissor2 = hardwareMap.dcMotor.get("scissor2");
        pinion = hardwareMap.dcMotor.get("pinion");

        claw = hardwareMap.get(Servo.class, "claw");
        foundation1 = hardwareMap.get(Servo.class, "foundation1");
        foundation2 = hardwareMap.get(Servo.class, "foundation2");

        scissor_touch = hardwareMap.touchSensor.get("scissor_touch");

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
            scissor_power = gamepad2.right_stick_y;
            if(scissor_touch.isPressed() == true && scissor_power > 0) {
                scissor_power = 0;
            }
            pinion_power = gamepad2.left_stick_y;

            scissor1.setPower(scissor_power);
            scissor2.setPower(scissor_power);
            pinion.setPower(pinion_power);

            claw_open = gamepad2.left_bumper;
            claw_close = gamepad2.right_bumper;

            if(claw_open == true) {
                claw.setPosition(1);
            } else if(claw_close == true) {
                claw.setPosition(0);
            }

            //Foundation Grab and Move
             boolean grab = gamepad2.a;
             boolean release = gamepad2.b;
             if(grab == true) {
                 foundation1.setPosition(1);
                 foundation2.setPosition(1);
             } else if (release == true) {
                 foundation1.setPosition(0);
                 foundation2.setPosition(0);
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
