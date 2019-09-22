package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="testAuton")
public class testAuton extends LinearOpMode {
    //Define Motor
    DcMotor testMotor;
    ColorSensor sensorColor;
    //DigitalChannel testTouch;
    TouchSensor testTouch;



    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = hardwareMap.dcMotor.get("test");
        //testTouch = hardwareMap.get(DigitalChannel.class, "test_touch");
        //testTouch = hardwareMap.touchSensor.get("test_touch");
        //sensorColor = hardwareMap.colorSensor.get("sensor_color");


        //testTouch.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();
        resetStartTime();

        if(opModeIsActive()) {
            spinForRevolutions(10, 0.5);


        }
    }

    private void touchtest() {
        while(opModeIsActive()){
            if(testTouch.isPressed()) {
                testMotor.setPower(0);
            } else if (!testTouch.isPressed()) {
                testMotor.setPower(1);
            }
        }

    }

    public void spinForRevolutions(int revolutions, double speed) {

        //Start Driving with motors
        testMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        testMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        testMotor.setPower(speed);
        double targetCounts = revolutions*537.6;

        //Do comparison for reached distance or not
        if( speed <0) {
            while(testMotor.getCurrentPosition()> targetCounts) {
                sleep(5);
                telemetry.addData("  Counts", testMotor.getCurrentPosition());
                telemetry.addData("  Target", targetCounts);
                telemetry.update();
                if(opModeIsActive() == false) {
                    break;
                }
            }
        } else if( speed >0) {
            while(testMotor.getCurrentPosition()< targetCounts) {
                sleep(5);
                telemetry.addData("  Counts", testMotor.getCurrentPosition());
                telemetry.addData("  Target", targetCounts);
                telemetry.update();
                if(opModeIsActive() == false) {
                    break;
                }
            }
        }
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        testMotor.setPower(0);
    }

}