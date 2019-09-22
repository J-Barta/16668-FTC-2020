package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="mecanumAuton")
public class mecanumAuton extends LinearOpMode {
    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;



    @Override
    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.FORWARD);
        left_back.setDirection(DcMotorSimple.Direction.FORWARD);
        waitForStart();
        if(opModeIsActive()) {

            strafe(0.5, 2);

        }

    }
    public void strafe(double power, double revolutions){
        //Calculate Distance
        double counts = revolutions*537.6;

        //Prep motors to run
        resetEncoder();
        useEncoder();
        setPowers(-power, power, power, -power);
        /*
        if(power < 0) {


            //Left subroutine
            while(right_front.getCurrentPosition() < revolutions) {
                sleep(5);
                if(opModeIsActive() == false) {
                    break;
                }
            }
            setBrakeBehavior();
            setPowers(0,0,0,0);
        } else if (power > 0) {
            //right subroutine
            while(right_front.getCurrentPosition()> revolutions) {
                sleep(5);
                if(opModeIsActive() == false) {
                    break;
                }
            }
            setBrakeBehavior();
            setPowers(0,0,0,0);
        }
        */
         sleep(5000);


    }



    public void driveStraight(double Power, double millimeters) {
        double encoderCounts = (millimeters/307.867)*537.6;
        resetEncoder();
        useEncoder();
        setPowers(Power, Power, Power, Power);
        if(Power < 0) {
            encoderCounts = encoderCounts *-1;
            while(right_front.getCurrentPosition() > encoderCounts) {
                sleep(5);
                if(opModeIsActive() == false) {
                    break;
                }
            }
            setBrakeBehavior();
            setPowers(0,0,0,0);
        } else if(Power >0) {
            while(right_front.getCurrentPosition() < encoderCounts) {
                sleep(5);
                if(opModeIsActive() == false) {
                    break;
                }
            }
            setBrakeBehavior();
            setPowers(0,0,0,0);
        }

    }
    public void drivePrep() {
        resetEncoder();
        useEncoder();
    }
    public void resetEncoder() {
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void useEncoder() {
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setPowers(double right_front_power, double right_back_power, double left_front_power, double left_back_power) {
        right_front.setPower(right_front_power);
        right_back.setPower(right_back_power);
        left_front.setPower(left_front_power);
        left_back.setPower(left_back_power);
    }
    public void setBrakeBehavior() {
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}
