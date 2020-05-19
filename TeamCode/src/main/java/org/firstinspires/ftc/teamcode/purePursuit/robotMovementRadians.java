package org.firstinspires.ftc.teamcode.purePursuit;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Robot Movement Radians")
public class robotMovementRadians extends LinearOpMode {
    globalCoordinatePosition globalPositionUpdate;

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

        telemetry.addData(" Status", " Initializing");
        telemetry.update();

        initDriveHardwareMap(rfName, rbName, lfName, lbName,  verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData(" Status", " Waiting for Start");
        telemetry.update();

        globalPositionUpdate = new globalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        waitForStart();

        if(opModeIsActive()) {
            goToPosition(24, 0, 0.5, 0, 2, 0.3, false, 90);
        }


        globalPositionUpdate.stop();

    }

    /**
     *
     * @param x X point to go to
     * @param y Y point to go to
     * @param movementSpeed Maximum linear speed
     * @param preferredAngle Angle that you want the robot to move toward the point at
     * @param error How close to the point you want the robot to get
     * @param turnSpeed How fast the robot can turn
     * @param doAbsoluteTurn Do another turn at the end of the movement?
     * @param absoluteAngle To what angle should the absolute turn go?
     */
    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double error, double turnSpeed,  boolean doAbsoluteTurn, double absoluteAngle) {
        double distanceToTarget = Math.hypot(x-(globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH), y-(-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH));

        while(opModeIsActive() && distanceToTarget > error) {
            double robotX = -globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
            double robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
            double robotOrientation = Math.toRadians(mathFunctions.interpretAngle(globalPositionUpdate.returnOrientation()));

            distanceToTarget = Math.hypot(x-(robotX), y-(robotY));

            double absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);
            double relativeAngleToTarget = mathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            double relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

            double MovementPowerDenominator = Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint);
            double movementXPower = relativeXToPoint / MovementPowerDenominator;
            double movementYPower = relativeYToPoint / MovementPowerDenominator;

            double movement_x = movementXPower * movementSpeed;
            double movement_y = movementYPower * movementSpeed;

            double relativeTurnAngle = relativeAngleToTarget + Math.toRadians(90) + Math.toRadians(preferredAngle);
            double movement_turn = Range.clip(relativeTurnAngle/Math.toRadians(30), -1, 1) * turnSpeed;

            if(distanceToTarget < 5) {
                movement_turn = 0;
            }

            telemetry.addData( " x" , movement_x);
            telemetry.addData(" y", movement_y);
            telemetry.addData(" theta", movement_turn);
            telemetry.addData( " Distance to Target", distanceToTarget);
            telemetry.addData( " Absolute Angle to Target", Math.toDegrees(absoluteAngleToTarget));
            telemetry.addData( " Relative to Target", Math.toDegrees(relativeAngleToTarget));
            telemetry.addData( " Relative Turn Angle", Math.toDegrees(relativeTurnAngle));
            telemetry.addData(" xpos", robotX);
            telemetry.addData(" ypos", robotY);
            telemetry.addData(" orientation", robotOrientation);
            telemetry.update();


            //movement_x = 0;
            //movement_y = 0;
            movement_turn = 0;


            double leftFront = -movement_y - movement_x - movement_turn;
            double rightFront = movement_y - movement_x - movement_turn;
            double rightBack = movement_y + movement_x - movement_turn;
            double leftBack = -movement_y + movement_x - movement_turn;

            //Scale Motor Powers to preserve movement shape
            double lfAbs = Math.abs(leftFront);
            double rfAbs = Math.abs(rightFront);
            double rbAbs = Math.abs(rightBack);
            double lbAbs = Math.abs(leftBack);

            if(lfAbs > rfAbs && lfAbs > rbAbs && lfAbs > lbAbs && lfAbs > 1) {
                leftFront = leftFront/lfAbs;
                rightFront = rightFront/lfAbs;
                rightBack = rightBack/lfAbs;
                leftBack = leftBack/lfAbs;
            } else if(rfAbs > lfAbs && rfAbs > rbAbs && rfAbs > lbAbs && rfAbs > 1) {
                leftFront = leftFront/rfAbs;
                rightFront = rightFront/rfAbs;
                rightBack = rightBack/rfAbs;
                leftBack = leftBack/rfAbs;
            } else if(rbAbs > lfAbs && rbAbs > rfAbs && rbAbs > lbAbs && rbAbs > 1) {
                leftFront = leftFront/rbAbs;
                rightFront = rightFront/rbAbs;
                rightBack = rightBack/rbAbs;
                leftBack = leftBack/rbAbs;
            } else if(lbAbs > lfAbs && lbAbs > rfAbs && lbAbs > rbAbs && lbAbs > 1) {
                leftFront = leftFront/lbAbs;
                rightFront = rightFront/lbAbs;
                rightBack = rightBack/lbAbs;
                leftBack = leftBack/lbAbs;
            }

            leftFront = Range.clip(leftFront, -1, 1);
            rightFront = Range.clip(rightFront, -1, 1);
            rightBack = Range.clip(rightBack, -1, 1);
            leftBack = Range.clip(leftBack, -1, 1);

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


    }
    public double relativeXFinder(double relativeX) {
        return (-globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH) + relativeX;
    }
    public double relativeYFinder(double relativeY) {
        return (-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH) + relativeY;
    }
    public void initDriveHardwareMap(String rfName, String rbName, String lfName, String lbName, String vlEncoderName, String vrEncoderName, String hEncoderName){
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
