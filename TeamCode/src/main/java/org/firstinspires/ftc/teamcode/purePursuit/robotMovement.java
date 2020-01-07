package org.firstinspires.ftc.teamcode.purePursuit;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.purePursuit.globalCoordinatePosition;

import java.util.ArrayList;

public class robotMovement extends LinearOpMode {
    globalCoordinatePosition globalPositionUpdate;

    public static void followCurve(ArrayList<curvePoint> allPoints, double followAngle) {

    }

    public  curvePoint getFollowPointPath(ArrayList<curvePoint> pathPoints,Point robotLocation, double xPos, double yPos, double followRadius) {
        curvePoint followMe = new curvePoint(pathPoints.get(0));


        for(int i = 0; i < pathPoints.size() - 1; i++) {
            curvePoint startLine = pathPoints.get(i);
            curvePoint endLine = pathPoints.get(i+1);

            ArrayList<Point> intersections = mathFunctions.lineCircleIntersection(robotLocation, followRadius, startLine.toPoint(startLine), endLine.toPoint(endLine));


            double closestAngle = 1000000000;

            for(Point thisIntersection : intersections) {
                double angle = Math.atan2(thisIntersection.y - globalPositionUpdate.returnYCoordinate(), thisIntersection.x - globalPositionUpdate.returnXCoordinate());

                double deltaAngle = Math.abs(mathFunctions.AngleWrap(angle - globalPositionUpdate.returnOrientation()));

                if(deltaAngle < closestAngle) {
                    closestAngle = deltaAngle;
                    followMe.setPoint(thisIntersection);
                }
            }

        }
        return followMe;
    }

    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;
    DcMotor verticalLeft, verticalRight, horizontal;
    final double COUNTS_PER_INCH = 307.699557;


    @Override
    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

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

    /**
     *
     * @param x
     * @param y
     * @param movementSpeed
     * @param preferredAngle
     * @param turnSpeed
     */

    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed ) {
        double distanceToTarget = Math.hypot(x-globalPositionUpdate.returnXCoordinate(), y-globalPositionUpdate.returnYCoordinate());

        double absoluteAngleToTarget = Math.atan2(y-globalPositionUpdate.returnYCoordinate(), x-globalPositionUpdate.returnXCoordinate());

        double relativeAngleToPoint = mathFunctions.AngleWrap(absoluteAngleToTarget - (globalPositionUpdate.returnOrientation()-Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) *distanceToTarget;

        double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double movement_x = movementXPower*movementSpeed;
        double movement_y = movementYPower*movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        double movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;
        if(distanceToTarget < 5) {
            movement_turn = 0;
        }
        right_front.setPower(movement_y - movement_turn - movement_x);
        right_back.setPower(movement_y - movement_turn + movement_x);
        left_front.setPower(movement_y + movement_turn + movement_x);
        left_back.setPower(movement_y + movement_turn - movement_x);

        while(opModeIsActive()) {

        }

    }
}
