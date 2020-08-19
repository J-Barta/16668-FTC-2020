package org.firstinspires.ftc.teamcode.Odometry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import static java.lang.Math.abs;
import static java.lang.Math.toRadians;
import static org.firstinspires.ftc.teamcode.Odometry.mathFunctions.interpretAngle;


@Autonomous(name = "Robot Movement Test")
public class RobotMovementTest extends LinearOpMode {
    //Drive motors
    DcMotor right_front, right_back, left_front, left_back;
    //Odometry Wheels
    DcMotor verticalLeft, verticalRight, horizontal;

    final double COUNTS_PER_INCH = 307.699557;

    //Hardware Map Names for drive motors and odometry wheels. THIS WILL CHANGE ON EACH ROBOT, YOU NEED TO UPDATE THESE VALUES ACCORDINGLY
    String rfName = "right_front", rbName = "right_back", lfName = "left_front", lbName = "left_back";
    String verticalLeftEncoderName = rbName, verticalRightEncoderName = lfName, horizontalEncoderName = lbName;

    globalCoordinatePosition globalPositionUpdate;

    @Override
    public void runOpMode() throws InterruptedException {
        //Initialize hardware map values. PLEASE UPDATE THESE VALUES TO MATCH YOUR CONFIGURATION
        initDriveHardwareMap(rfName, rbName, lfName, lbName, verticalLeftEncoderName, verticalRightEncoderName, horizontalEncoderName);

        telemetry.addData("Status", "Init Complete");
        telemetry.update();
        waitForStart();

        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions
        globalPositionUpdate = new globalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        Thread positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
        globalPositionUpdate.reverseNormalEncoder();

        if (opModeIsActive()) {
            goToPosition(24, 0, 0.25, 90, 2, 0.3);
        }

        //Stop the thread
        globalPositionUpdate.stop();

    }

    /**
     * @param x X point to go to
     * @param y Y point to go to
     * @param movementSpeed Maximum linear speed
     * @param preferredAngle Angle that you want the robot to move toward the point at
     * @param error How close to the point you want the robot to get (in inches)
     * @param turnSpeed How fast the robot can turn
     */

    public void goToPosition(double x, double y, double movementSpeed, double preferredAngle, double error, double turnSpeed) {
        double distanceToTarget = Math.hypot(x-(globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH), y-(-globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH));

        while(opModeIsActive() && distanceToTarget > error) {
            double robotX = globalPositionUpdate.returnXCoordinate()/COUNTS_PER_INCH;
            double robotY = -globalPositionUpdate.returnYCoordinate()/COUNTS_PER_INCH;
            double robotOrientation = toRadians(interpretAngle(globalPositionUpdate.returnOrientation()));

            distanceToTarget = Math.hypot(x-(robotX), y-(robotY));

            double absoluteAngleToTarget = Math.atan2(y-robotY, x-robotX);
            double relativeAngleToTarget = mathFunctions.AngleWrap(absoluteAngleToTarget - (robotOrientation));

            double relativeXToPoint = Math.cos(relativeAngleToTarget) * distanceToTarget;
            double relativeYToPoint = Math.sin(relativeAngleToTarget) * distanceToTarget;

            double MovementPowerDenominator = abs(relativeXToPoint) + abs(relativeYToPoint);
            double movementXPower = relativeXToPoint / MovementPowerDenominator;
            double movementYPower = relativeYToPoint / MovementPowerDenominator;

            double movement_x = movementXPower * movementSpeed;
            double movement_y = movementYPower * movementSpeed;

            double relativeTurnAngle = relativeAngleToTarget - toRadians(180) + toRadians(preferredAngle);
            double movement_turn = Range.clip(relativeTurnAngle/ toRadians(30), -1, 1) * turnSpeed;

            if(distanceToTarget < 5) {
                movement_turn = 0;
            }


            //Lots of Telemetry. Remove some of this.

            //Display Global (x, y, theta) coordinates
            telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Y Position", -globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
            telemetry.addData("Orientation (Degrees)", interpretAngle(globalPositionUpdate.returnOrientation()));
            telemetry.addData("Unchanged Orientation", globalPositionUpdate.returnOrientation());
            telemetry.addData("Vertical left encoder position", verticalLeft.getCurrentPosition());
            telemetry.addData("Vertical right encoder position", verticalRight.getCurrentPosition());
            telemetry.addData("horizontal encoder position", horizontal.getCurrentPosition());
            telemetry.update();

            //Testing values
            movement_x = 0;
            movement_y = 0;
            movement_turn = 0;


            double leftFront = -movement_y - movement_x + movement_turn;
            double rightFront = movement_y - movement_x + movement_turn;
            double rightBack = movement_y + movement_x + movement_turn;
            double leftBack = -movement_y + movement_x + movement_turn;

            //Scale Motor Powers to preserve movement shape
            double lfAbs = abs(leftFront);
            double rfAbs = abs(rightFront);
            double rbAbs = abs(rightBack);
            double lbAbs = abs(leftBack);

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


        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_front.setDirection(DcMotorSimple.Direction.REVERSE);
        right_back.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Hardware Map Init Complete");
        telemetry.update();
    }
}
