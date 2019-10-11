package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="scissorJack", group="TeleOp")
public class scissorJack extends LinearOpMode {

    public DcMotor scissor1;
    public DcMotor scissor2;
    @Override
    public void runOpMode() throws InterruptedException {
        scissor1 = hardwareMap.dcMotor.get("scissor1");
        scissor2 = hardwareMap.dcMotor.get("scissor2");
        waitForStart();
        while(opModeIsActive()) {
            double scissor_power =gamepad1.right_stick_y;

            scissor1.setPower(scissor_power);
            scissor2.setPower(scissor_power);
        }



    }
}