package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name = "colorReaderRGB")
public class colorReaderRGB extends LinearOpMode {

    ColorSensor sensorColor;

    @Override
    public void runOpMode() throws InterruptedException {
        sensorColor = hardwareMap.colorSensor.get("sensor_color");

        double r;
        double g;
        double b;

        waitForStart();
        resetStartTime();

        while(opModeIsActive()) {
            r = sensorColor.red();
            b = sensorColor.green();
            g = sensorColor.blue();
            telemetry.addData("r", r);
            telemetry.addData("g", g);
            telemetry.addData("b", b);

            if(r>g && r>b) {
                telemetry.addData("  Current Color", " Red");
            } else if(g>r && g>b) {
                telemetry.addData("  Current Color", " Green");
            } else if(b>r && b>g) {
                telemetry.addData("  Current Color", " Blue");
            } else {
                telemetry.addData("  Current Color", " Unclear");
            }

            telemetry.update();
        }

    }
}
