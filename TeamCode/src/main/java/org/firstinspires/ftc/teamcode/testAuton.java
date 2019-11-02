package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="testAuton")
public class testAuton extends LinearOpMode {
    public DistanceSensor distance;




    @Override
    public void runOpMode() throws InterruptedException {
        distance = hardwareMap.get(DistanceSensor.class, "distance");


        waitForStart();
        resetStartTime();

        while(opModeIsActive()) {
            telemetry.addData("", distance.getDistance(DistanceUnit.MM));
            telemetry.update();


        }
    }



}