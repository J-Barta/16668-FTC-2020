package org.firstinspires.ftc.teamcode.PreviousAutonsScissorBot;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Blue Quarry Auton")
public class quarryAutonBlue2Stones extends LinearOpMode {
    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;
    public DcMotor scissor1;
    public DcMotor scissor2;
    public DcMotor pinion;
    public DcMotor tape;


    public BNO055IMU imu;
    public DistanceSensor stone_distance;
    public ColorSensor left_color;
    public ColorSensor right_color;

    public Servo claw;
    public Servo foundation1;
    public Servo foundation2;

    public TouchSensor scissor_touch;

    double current;
    boolean first = true;
    boolean firstArm = true;
    double startTime = 0;
    boolean done = false;
    boolean done2 = false;
    boolean done3 = false;
    boolean forfeit = false;




    @Override
    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");
        scissor1 = hardwareMap.dcMotor.get("scissor1");
        scissor2 = hardwareMap.dcMotor.get("scissor2");
        pinion = hardwareMap.dcMotor.get("pinion");
        tape = hardwareMap.dcMotor.get("tape");

        claw = hardwareMap.servo.get("claw");
        foundation1 = hardwareMap.get(Servo.class, "foundation1");
        foundation2 = hardwareMap.get(Servo.class, "foundation2");

        stone_distance = hardwareMap.get(DistanceSensor.class, "stone_distance");
        left_color = hardwareMap.get(ColorSensor.class, "left_color");
        right_color = hardwareMap.get(ColorSensor.class, "right_color");

        scissor_touch = hardwareMap.touchSensor.get("scissor_touch");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        right_front.setDirection(DcMotorSimple.Direction.FORWARD);
        right_back.setDirection(DcMotorSimple.Direction.FORWARD);
        left_front.setDirection(DcMotorSimple.Direction.REVERSE);
        left_back.setDirection(DcMotorSimple.Direction.REVERSE);
        scissor1.setDirection(DcMotorSimple.Direction.REVERSE);
        scissor2.setDirection(DcMotorSimple.Direction.REVERSE);
        pinion.setDirection(DcMotorSimple.Direction.REVERSE);

        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        scissor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pinion.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        if (opModeIsActive()) {
            claw.setPosition(1);
            driveAndSetArm();
            distanceDrive(-0.125, 75);
            senseAndGrab();
            sleep(5000);
        }

    }
    public void driveAndSetArm() {
        double encoderCounts = (600/307.867) * 537.6;
        pinion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pinion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scissor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        scissor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        resetEncoder();
        useEncoder();
        pinion.setPower(-1);
        setPowers(-0.25, -0.25, -0.25, -0.25);
        while (right_front.getCurrentPosition() > -encoderCounts  || done3 == false) {
            sleep(5);
            if (pinion.getCurrentPosition() < -288*1) {
                pinion.setPower(0);
                done = true;
                pinion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pinion.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }
            if (done) {
                if(firstArm) {
                    scissor1.setPower(1);
                    scissor2.setPower(1);
                    firstArm = false;
                }

                if (scissor_touch.isPressed() == true) {
                    scissor1.setPower(0);
                    scissor2.setPower(0);
                    done2 = true;
                }
            }
            if (done2) {
                if (first) {
                    first = false;
                    startTime = getRuntime();
                    pinion.setPower(1);
                }

                if (getRuntime() - startTime >= 0.5 && startTime != 0) {
                    pinion.setPower(0);
                    done3 = true;
                }
            }
            if(right_front.getCurrentPosition() <= -encoderCounts) {
                setPowers(0,0,0,0);
            }
            if(opModeIsActive() == false) {
                break;
            }
            scissorCheck();
        }
        setPowers(0,0,0,0);
    }
    public void senseAndGrab () {
        float[] hsv_left = new float[3];
        float[] hsv_right = new float[3];
        Color.RGBToHSV(left_color.red(), left_color.green(), left_color.blue(), hsv_left);
        Color.RGBToHSV(right_color.red(), right_color.green(), right_color.blue(), hsv_right);
        if(hsv_left[0]-hsv_right[0] <-8 && forfeit == false) {
            //Right
            strafe(-0.4, 0.8);
            distanceDrive(-0.25, 75);
            if(forfeit==false){
                moveArm(-1, 1);
                claw.setPosition(0);
                sleep(600);
                driveStraight(0.25, 100);
                turn(0.25, 82);
                driveStraight(-0.5, 1500);
                claw.setPosition(1);
                sleep(300);
                //strafe(0.4,0.25);
                driveAndArm(1750, 0.35, 1, 1);
                driveStraight(0.25, 300);
                driveStraight(-0.25, 255);
                turn(-0.27, -88);
                strafe(0.4, 1.25);
                claw.setPosition(0);
                sleep(600);
                strafe(-0.4, 1.25);
                turn(0.27, 82);
                driveStraight(-0.5,1620);
                claw.setPosition(1);
                sleep(300);
                moveTapeandRetract(-1, 24, 0);
                //driveStraight(0.25, 250);
            }
        }else if(hsv_left[0]-hsv_right[0] > 8&& forfeit== false) {
            //Left
            driveStraight(0.25, 25);
            strafe(0.4, 0.6);
            moveArm(-1, 0.5);
            claw.setPosition(0);
            sleep(600);
            driveStraight(0.25, 100);
            turn(0.25, 83);
            driveStraight(-0.5, 920);
            claw.setPosition(1);
            sleep(600);
            driveAndArm(1447, 0.5, 0.5, 1);
            turn(-0.25, 0);
            distanceDrive(-0.25, 75);
            if(forfeit==false) {
                moveArm(-1, 1);
                claw.setPosition(0);
                sleep(600);
                moveArm(1, 0.5);
                driveStraight(0.25, 100);
                turn(0.25, 84);
                driveStraight(-0.5,1600);
                claw.setPosition(1);
                sleep(600);
                moveTapeandRetract(-1, 24, 0.25);
                /*
                driveStraight(0.25, 250);
                moveArm(1, 0.5);
                 */
            }
        }else{
            if(forfeit == false) {
                //Middle
                moveArm(-1, 1.5);
                claw.setPosition(0);
                sleep(600);
                moveArm(1, 1);
                driveStraight(0.25, 100);
                turn(0.25, 83);
                driveStraight(-0.5, 1240);
                claw.setPosition(1);
                sleep(600);
                driveAndArm(1750,0.5,0.5,1);
                turn(-0.25, 0);
                distanceDrive(-0.125, 75);
                if(forfeit== false) {
                    moveArm(-1, 1);
                    claw.setPosition(0);
                    sleep(600);
                    moveArm(1, 0.5);
                    driveStraight(0.25, 100);
                    turn(0.25, 83);
                    driveStraight(-0.5, 1800);
                    claw.setPosition(1);
                    sleep(600);
                    moveTapeandRetract(-1, 24, 0.25);
                    /*
                    driveStraight(0.25, 250);
                    moveArm(1, 0.5);
                     */
                }

            }
        }

    }
    public void moveTape(double power, double in) {
        double encoderCounts = (in/11.131625) * 288;
        tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tape.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        tape.setPower(power);
        if(power > 0) {
            while(tape.getCurrentPosition() < encoderCounts && opModeIsActive()) {
                sleep(5);
                scissorCheck();
            }
        } else if (power < 0 ) {
            while(tape.getCurrentPosition() > -encoderCounts && opModeIsActive()) {
                sleep(5);
                scissorCheck();
            }
        }
        tape.setPower(0);

    }
    public void moveTapeandRetract(double power, double in, double pinionTime) {
        double encoderCounts = (in/11.131625) * 288;
        tape.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        tape.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinion.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pinion.setPower(1);
        tape.setPower(power);
        double startTime = getRuntime();
        if(power > 0) {
            while(tape.getCurrentPosition() < encoderCounts && opModeIsActive()) {
                sleep(5);
                scissorCheck();
                if(getRuntime()-startTime > pinionTime) {
                    pinion.setPower(0);
                }
            }
        } else if (power < 0 ) {
            while(tape.getCurrentPosition() > -encoderCounts && opModeIsActive()) {
                sleep(5);
                scissorCheck();
                if(getRuntime()-startTime > pinionTime) {
                    pinion.setPower(0);
                }
            }
        }
        tape.setPower(0);

    }
    public void driveAndArm(double mm, double wheelPower, double armRotations, double armPower) {
        double encoderCounts = (mm/307.867)*537.6;
        pinion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pinion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetEncoder();
        useEncoder();
        pinion.setPower(armPower);
        setPowers(wheelPower, wheelPower, wheelPower, wheelPower);
        if(wheelPower>0 && armPower >0) {
            while(right_front.getCurrentPosition() < encoderCounts && opModeIsActive()) {
                sleep(5);
                if(pinion.getCurrentPosition() >= 288*armRotations) {
                    pinion.setPower(0);
                }
                telemetry.addData("", pinion.getCurrentPosition());
                telemetry.update();
            }
        } else if (wheelPower > 0 && armPower <0) {
            while(right_front.getCurrentPosition() < encoderCounts && opModeIsActive()) {
                sleep(5);
                if(pinion.getCurrentPosition() <= -288*armRotations) {
                    pinion.setPower(0);
                }
            }
        } else if(wheelPower < 0 && armPower >0) {
            while (right_front.getCurrentPosition() > -encoderCounts && opModeIsActive()) {
                sleep(5);
                if (pinion.getCurrentPosition() >= 288 * armRotations) {
                    pinion.setPower(0);
                }
            }

        } else if(wheelPower < 0 && armPower < 0) {
            while(right_front.getCurrentPosition() > -encoderCounts && opModeIsActive()) {
                sleep(5);
                if(pinion.getCurrentPosition() <= -288*armRotations) {
                    pinion.setPower(0);
                }
            }
        }
        setPowers(0,0,0,0);
    }
    public void moveArm(double power, double revolutions) {
        pinion.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pinion.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pinion.setPower(power);
        if(power < 0) {
            while(pinion.getCurrentPosition() > -288*revolutions && opModeIsActive()) {
                sleep(5);
            }
        } else if (power > 0) {
            while(pinion.getCurrentPosition() < 288*revolutions && opModeIsActive())  {
                sleep(5);
            }
        }

        pinion.setPower(0);
    }

    public void distanceDrive(double power, double distanceTo) {
        resetEncoder();
        useEncoder();
        setPowers(power, power, power ,power);
        while(stone_distance.getDistance(DistanceUnit.MM) >= distanceTo && opModeIsActive()) {
            if(right_front.getCurrentPosition() < -537.6*2.5) {
                forfeit=true;
                break;
            }
        }
        setPowers(0,0,0,0);
    }


    public void turn(double power, double degrees) {
        Orientation turn = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        setPowers(-power, -power, power, power);

        if(power > 0) {
            current = turn.firstAngle;
            while (current <= degrees) {
                turn = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current = turn.firstAngle;
                if (opModeIsActive() == false) {
                    break;
                }
            }
        } else if(power <0) {
            current = turn.firstAngle;
            while(current >= degrees) {
                turn = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current = turn.firstAngle;
                if (opModeIsActive() == false) {
                    break;
                }
            }
        }
        setBrakeBehavior();
        setPowers(0,0,0,0);
    }
    public void strafe(double power, double revolutions){
        //Calculate Distance
        double counts = revolutions*537.6;

        //Prep motors to run
        resetEncoder();
        useEncoder();
        setPowers(-power, power, power, -power);

        if(power < 0) {

            //Left subroutine
            while(right_front.getCurrentPosition() < counts) {
                sleep(5);
                if(opModeIsActive() == false) {
                    break;
                }
            }
            setBrakeBehavior();
            setPowers(0,0,0,0);
        } else if (power > 0) {
            counts = counts * -1;
            //right subroutine
            while(right_front.getCurrentPosition()> counts) {
                sleep(5);
                if(opModeIsActive() == false) {
                    break;
                }
            }
            setBrakeBehavior();
            setPowers(0,0,0,0);
        }




    }



    public void driveStraight(double Power, double millimeters) {
        double encoderCounts = (millimeters/307.867)*537.6;
        resetEncoder();
        useEncoder();
        setPowers(Power, Power, Power, Power);
        if(Power < 0) {
            encoderCounts = encoderCounts *-1;
            while(right_front.getCurrentPosition() > encoderCounts) {
                sleep(5);
                if(opModeIsActive() == false) {
                    break;
                }
            }
            setBrakeBehavior();
            setPowers(0,0,0,0);
        } else if(Power >0) {
            while(right_front.getCurrentPosition() < encoderCounts) {
                sleep(5);
                if(opModeIsActive() == false) {
                    break;
                }
            }
            setBrakeBehavior();
            setPowers(0,0,0,0);
        }

    }
    public void drivePrep() {
        resetEncoder();
        useEncoder();
    }
    public void resetEncoder() {
        right_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_front.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        left_back.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void useEncoder() {
        right_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_front.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        left_back.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void setPowers(double right_front_power, double right_back_power, double left_front_power, double left_back_power) {
        right_front.setPower(right_front_power);
        right_back.setPower(right_back_power);
        left_front.setPower(left_front_power);
        left_back.setPower(left_back_power);
    }
    public void setBrakeBehavior() {
        right_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_front.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left_back.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void scissorCheck() {
        if(scissor1.getPower() > 0 && scissor_touch.isPressed()) {
            scissor1.setPower(0);
            scissor2.setPower(0);
        }
    }
}
