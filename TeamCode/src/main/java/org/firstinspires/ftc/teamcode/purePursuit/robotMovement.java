package org.firstinspires.ftc.teamcode.purePursuit;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Odometry.GlobalCoordinatePositionUpdateSample;


@Autonomous(name= "Test Odometry Drive")
public class robotMovement extends LinearOpMode {
    globalCoordinatePosition globalPositionUpdate;
    /*
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

     */
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = lbName;

    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;

    DcMotor verticalLeft, verticalRight, horizontal;
    final double COUNTS_PER_INCH = 307.699557;


    @Override
    public void runOpMode() throws InterruptedException {
        /*
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");

        verticalEncoderLeft = hardwareMap.dcMotor.get("right_back");
        verticalEncoderRight = hardwareMap.dcMotor.get("left_front");
        horizontalEncoder = hardwareMap.dcMotor.get("left_back");

         */
        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        initDriveHardwareMap(rfName, rbName, lfName, lbName,  verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData(" Status", " Waiting for Start");
        telemetry.update();
        waitForStart();
        globalPositionUpdate = new globalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        if(opModeIsActive()) {
            goToPosition(0, 24, 0.25,  0, 0.3, false, 90);
            //goToPosition(12, -24, -0.25,  90, 0.3, false, 90);
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
     * @param doAbsoluteTurn
     * @param absoluteAngle
     */



    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double turnSpeed,  boolean doAbsoluteTurn, double absoluteAngle) {
        /*
        This should calculate the initial movements that are necessary for arriving at the point
         */

        double distanceToTarget = Math.hypot(x-(globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH), y-(globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH));

        double absoluteAngleToTarget = Math.atan2(y-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH, x-globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);

        double relativeAngleToPoint = mathFunctions.AngleWrap(absoluteAngleToTarget - (mathFunctions.interpretAngle(globalPositionUpdate.returnOrientation())));
        double relativeAngleForXY = mathFunctions.AngleWrap(absoluteAngleToTarget - (globalPositionUpdate.returnOrientation()));

        double relativeXToPoint = Math.cos(relativeAngleForXY) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleForXY) *distanceToTarget;

        double movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double movement_x = movementXPower*movementSpeed;
        double movement_y = movementYPower*movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint + preferredAngle;
        double movement_turn = Range.clip(relativeTurnAngle/30, -1, 1) * turnSpeed;
        if(distanceToTarget < 1) {
            movement_turn = 0;
        }

        double leftFront = -movement_y - movement_x - movement_turn;
        double rightFront = movement_y - movement_x - movement_turn;
        double rightBack = movement_y + movement_x - movement_turn;
        double leftBack = -movement_y + movement_x - movement_turn;

        leftFront = Range.clip(leftFront, -1, 1);
        rightFront = Range.clip(rightFront, -1, 1);
        rightBack = Range.clip(rightBack, -1, 1);
        leftBack = Range.clip(leftBack, -1, 1);

        right_front.setPower(rightFront);
        right_back.setPower(rightBack);
        left_front.setPower(leftFront);
        left_back.setPower(leftBack);
        /*
        This loop constantly updates the position and returns new motor powers for use. Once the robot is close enough to the point,
        the robot should stop.
         */
        while(opModeIsActive() && distanceToTarget > 1) {

            distanceToTarget = Math.hypot(x-(globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH), y-(globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH));

            absoluteAngleToTarget = Math.atan2(y-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH, x-globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH);

            relativeAngleToPoint = mathFunctions.AngleWrap(absoluteAngleToTarget - (mathFunctions.interpretAngle(globalPositionUpdate.returnOrientation())));
            relativeAngleForXY = mathFunctions.AngleWrap(absoluteAngleToTarget - (globalPositionUpdate.returnOrientation()));


            relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
            relativeYToPoint = Math.sin(relativeAngleToPoint) *distanceToTarget;

            movementXPower = relativeXToPoint/ (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
            movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

            movement_x = movementXPower*movementSpeed;
            movement_y = movementYPower*movementSpeed;

            movement_x = 0;


            relativeTurnAngle = relativeAngleToPoint + preferredAngle;

            movement_turn = Range.clip(relativeTurnAngle/30, -1, 1)*turnSpeed;

            movement_turn = 0;
            if(distanceToTarget < 1) {
                movement_turn = 0;
            }

            telemetry.addData( " x" , movement_x);
            telemetry.addData(" y", movement_y);
            telemetry.addData(" theta", movement_turn);
            telemetry.addData(" Orientation", globalPositionUpdate.returnOrientation());
            telemetry.addData(" Distance to Target", distanceToTarget);

            leftFront = -movement_y - movement_x - movement_turn;
            rightFront = movement_y - movement_x - movement_turn;
            rightBack = movement_y + movement_x - movement_turn;
            leftBack = -movement_y + movement_x - movement_turn;

            leftFront = Range.clip(leftFront, -1, 1);
            rightFront = Range.clip(rightFront, -1, 1);
            rightBack = Range.clip(rightBack, -1, 1);
            leftBack = Range.clip(leftBack, -1, 1);

            telemetry.addData(" lf", leftFront);
            telemetry.addData(" rf", rightFront);
            telemetry.addData(" rb", rightBack);
            telemetry.addData(" lb", leftBack);
            telemetry.update();

            right_front.setPower(rightFront);
            right_back.setPower(rightBack);
            left_front.setPower(leftFront);
            left_back.setPower(leftBack);
        }

        //Turns motors off
        right_front.setPower(0);
        right_back.setPower(0);
        left_front.setPower(0);
        left_back.setPower(0);
        /*
        if(doAbsoluteTurn == true) {
            //Calculates the angle away from the target absolute angle
            double angleFromTarget = Math.abs((globalPositionUpdate.returnOrientation()-90) - absoluteAngle);

            while(opModeIsActive() && angleFromTarget > 2) {
                relativeAngleToPoint = mathFunctions.AngleWrap(absoluteAngle - (globalPositionUpdate.returnOrientation()-90));
                relativeTurnAngle = relativeAngleToPoint - 180 + absoluteAngle;
                movement_turn = Range.clip(relativeTurnAngle/30, -1, 1) * turnSpeed;

                rightFront = -movement_turn;
                rightBack  = -movement_turn;
                leftFront  = movement_turn;
                leftBack   = movement_turn;

                right_front.setPower(rightFront);
                right_back.setPower(rightBack);
                left_front.setPower(leftFront);
                left_back.setPower(leftBack);

            }

            right_front.setPower(0);
            right_back.setPower(0);
            left_front.setPower(0);
            left_back.setPower(0);

        }

         */

    }
    private void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
        right_front = hardwareMap.dcMotor.get(rfName);
        right_back = hardwareMap.dcMotor.get(rbName);
        left_front = hardwareMap.dcMotor.get(lfName);
        left_back = hardwareMap.dcMotor.get(lbName);

        verticalLeft = hardwareMap.dcMotor.get(vlEncoderName);
        verticalRight = hardwareMap.dcMotor.get(vrEncoderName);
        horizontal = hardwareMap.dcMotor.get(hEncoderName);

        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        right_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }

}
