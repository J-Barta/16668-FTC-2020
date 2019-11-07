package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "testDrive", group = "TeleOp")
@Disabled

public class factoredDrive extends LinearOpMode {
    //Motor init
    DcMotor testMotor;
    double powerFactor = 1;
    boolean currentState;
    boolean prevState;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("  Robot status", "Initializing");
        telemetry.update();

        testMotor = hardwareMap.dcMotor.get("test_motor");

        telemetry.addData("  Robot status", "Initialized");
        telemetry.update();

        waitForStart();
        resetStartTime();

        telemetry.addData("  Robot status", "Robot is running");
        telemetry.update();

        while(opModeIsActive()) {
            double testPower = gamepad1.left_stick_y * powerFactor;

            testMotor.setPower(testPower);
            telemetry.addData("  Motor Power", testPower);
            telemetry.addData("  power Factor", powerFactor);
            telemetry.update();

            boolean powerHalf = gamepad1.right_bumper;
            boolean powerDouble = gamepad1.left_bumper;


            powerModifier(powerHalf, powerDouble);

        }



    }

    private void powerModifier(boolean powerHalf, boolean powerDouble) {
        if(powerDouble == true) {
            currentState = true;
        } else if (powerHalf == true) {
            currentState = true;
        } else {
            currentState = false;
        }
        if(currentState != prevState) {
            if(powerHalf == true && powerFactor > 0.125) {
                powerFactor = powerFactor/2;
                powerHalf = false;
            }
            if(powerDouble == true && powerFactor <1) {
                powerFactor = powerFactor*2;
                powerDouble = false;
            }
        }
        prevState = currentState;
    }

}
