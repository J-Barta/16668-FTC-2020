package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name= "FirstAuton")
public class FirstAuton extends LinearOpMode {
    //Define Motors
    DcMotor rightMotor;
    DcMotor leftMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        leftMotor = hardwareMap.dcMotor.get("left_motor");


        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        leftMotor.setDirection(DcMotor.Direction.FORWARD);


        waitForStart();
        resetStartTime();

        while(opModeIsActive()) {

            DriveForward(60);
        }




    }
    /*Drive Forward Function
    Set Distance with the inches param
     */
    public void DriveForward(double inches){
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double distance = inches/(4*3.14159265)*537.6;

        rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        rightMotor.setPower(0.5);
        leftMotor.setPower(0.5);

        while(rightMotor.getCurrentPosition() < distance) {
            sleep(10);
            if(opModeIsActive() == false) {
                break;
            }
        }

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightMotor.setPower(0);
        leftMotor.setPower(0);


    }

}
