package org.firstinspires.ftc.teamcode.purePursuit;

public class TestAutonOdometry extends robotMovementRadians {

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
            goToPosition(-24, 24, 0.5, 0, 2, 0.3, false, 90);
            goToPosition(-24 , -24, 0.5, 0, 2, 0.3, false, 90);
            goToPosition(0, 0, 0.4, 0, 0.5, 0.3, false, 90);
            goToPosition(-12, 0, 0.4, 0, 0.5, 0.3, false, 90);
            goToPosition(0, 0, 0.4, 0, 0.5, 0.3, false, 90);
        }


        globalPositionUpdate.stop();

    }
}
