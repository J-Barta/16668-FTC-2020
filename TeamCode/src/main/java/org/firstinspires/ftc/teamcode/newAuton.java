package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name="newAuton")
public class newAuton extends LinearOpMode {
    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;
    public void runOpMode() throws InterruptedException {
        right_back=hardwareMap.dcMotor.get ("right_back");
        right_front=hardwareMap.dcMotor.get ("right_front");
        left_front=hardwareMap.dcMotor.get ("left_front");
        left_back=hardwareMap.dcMotor.get ("left_back");

        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_front.setDirection(DcMotorSimple.Direction.FORWARD);
        left_back.setDirection(DcMotorSimple.Direction.FORWARD);



        waitForStart();

        if(opModeIsActive()) {
            driveForward(1000, 0.5);

        }
    }

    private void driveForward(double millimeters, double power) {
        double encodercounts=millimeters/307.867*537.6;
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        right_back.setPower(power);
        right_front.setPower(power);
        left_front.setPower(power);
        left_back.setPower(power);

        while(right_front.getCurrentPosition() < encodercounts) {
            sleep(5);
            if(opModeIsActive() == false) {
                break;
            }
        }
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        right_back.setPower(0);
        right_front.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
    }
}
