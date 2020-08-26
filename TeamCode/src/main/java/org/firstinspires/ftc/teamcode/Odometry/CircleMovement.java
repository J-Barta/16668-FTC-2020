package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import static java.lang.Math.*;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Odometry.mathFunctions.interpretAngle;

@Autonomous(name="Circle")
public class CircleMovement extends RobotMovement {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        initDriveHardwareMap(rfName, rbName, lfName, lbName,  verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        globalPositionUpdate = new globalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        telemetry.addData(" Status", " Waiting for Start");
        telemetry.update();

        waitForStart();

        if(opModeIsActive()) {
            circle(10, 0.5, 0, 0.5, 0.3);
            sleep(500);
        }


        globalPositionUpdate.stop();

    }

    public void circle(double radius, double movementSpeed, double preferredAngle, double error, double turnSpeed) {

        double originalX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
        double originalY = globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;

        double robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
        double robotY = globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;

        turnToPosition(robotX, robotY+1, turnSpeed);

        goToPosition(cos(0)*radius, sin(0)*radius, movementSpeed, preferredAngle, error, turnSpeed);

        turnToPositionNoStop(robotX+1, robotY+1, turnSpeed);

        turnToPosition(cos(toRadians(((double) 0/360)*360d))*radius, sin(((double) 90/360)*360d)*radius, turnSpeed);

        final Point[] points = new Point[360];

        for (int i=0; i <= 360; i++) {
            final double angle = toRadians(((double) i/360)*360d);
            goToPosition(cos(angle)*radius, sin(angle)*radius, movementSpeed, preferredAngle, error, turnSpeed);
        }

        turnAndGo(originalX, originalY, movementSpeed, preferredAngle, error, turnSpeed, turnSpeed);
        
        turnToPosition(robotX, robotY+1, turnSpeed);
    }
}
