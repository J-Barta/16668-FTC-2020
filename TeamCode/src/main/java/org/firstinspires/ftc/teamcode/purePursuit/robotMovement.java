package org.firstinspires.ftc.teamcode.purePursuit;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.purePursuit.globalCoordinatePosition;

public class robotMovement extends LinearOpMode {
    globalCoordinatePosition globalPositionUpdate;

    DcMotor verticalLeft, verticalRight, horizontal;
    final double COUNTS_PER_INCH = 307.699557;


    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        globalPositionUpdate = new globalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        if(opModeIsActive()) {
            goToPosition(358/2, 358/2, 0.5,  Math.toRadians(90), 0.3);
        }


        globalPositionUpdate.stop();

    }


    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed ) {
        double distanceToTarget = Math.hypot(x-globalPositionUpdate.returnXCoordinate(), y-globalPositionUpdate.returnYCoordinate());

        double absoluteAngleToTarget = Math.atan2(y-globalPositionUpdate.returnYCoordinate(), x-globalPositionUpdate.returnXCoordinate());

        double relativeAngleToPoint = mathFunctions.AngleWrap(absoluteAngleToTarget - (globalPositionUpdate.returnOrientation()-Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) *distanceToTarget;

        double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeYToPoint) + Math.abs(relativeYToPoint));

        double movement_x = movementXPower*movementSpeed;
        double movement_y = movementYPower*movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        double movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;
        /*
        motor[frontRight] = movement_y - movement_turn - movement_x;
        motor[backRight] =  movement_y - movement_turn + movement_x;
        motor[frontLeft] = movement_y + movement_turn + movement_x;
        motor[backLeft] =  movement_y + movement_turn - movement_x;

         */
        while(opModeIsActive()) {

        }

    }
}
