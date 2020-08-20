package org.firstinspires.ftc.teamcode.Odometry;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Test Odometry Autonomous")
public class TestAutonOdometry extends RobotMovementRadiansOld {

   @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        initDriveHardwareMap(rfName, rbName, lfName, lbName,  verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        globalPositionUpdate = new globalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

       telemetry.addData(" Status", " Waiting for Start");
       telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            goToPosition(-24, 24, 0.5, 0, 2, 0.3);
            goToPosition(-24 , -24, 0.5, 0, 2, 0.3);
            goToPosition(0, 0, 0.4, 0, 0.5, 0.3);
            goToPosition(-12, 0, 0.4, 0, 0.5, 0.3);
            goToPosition(0, 0, 0.4, 0, 0.5, 0.3);
        }


        globalPositionUpdate.stop();

    }
}
