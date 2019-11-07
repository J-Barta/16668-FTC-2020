package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Quarry Auton")
public class quarryAuton extends LinearOpMode {
    public DcMotor right_front;
    public DcMotor right_back;
    public DcMotor left_front;
    public DcMotor left_back;
    public DcMotor scissor1;
    public DcMotor scissor2;
    public DcMotor pinion;


    public BNO055IMU imu;
    public DistanceSensor stone_distance;
    public ColorSensor left_color;
    public ColorSensor right_color;

    public Servo claw;
    public Servo foundation1;
    public Servo foundation2;

    double current;




    @Override
    public void runOpMode() throws InterruptedException {
        right_front = hardwareMap.dcMotor.get("right_front");
        right_back = hardwareMap.dcMotor.get("right_back");
        left_front = hardwareMap.dcMotor.get("left_front");
        left_back = hardwareMap.dcMotor.get("left_back");
        scissor1 = hardwareMap.dcMotor.get("scissor1");
        scissor2 = hardwareMap.dcMotor.get("scissor2");
        pinion = hardwareMap.dcMotor.get("pinion");

        claw = hardwareMap.servo.get("claw");
        foundation1 = hardwareMap.get(Servo.class, "foundation1");
        foundation2 = hardwareMap.get(Servo.class, "foundation2");



        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        stone_distance = hardwareMap.get(DistanceSensor.class, "stone_distance");
        left_color = hardwareMap.get(ColorSensor.class, "left_color");
        right_color = hardwareMap.get(ColorSensor.class, "right_color");


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


        }


    }


    public void turn(double power, double degrees) {
        Orientation turn = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        setPowers(-power, -power, power, power);

        if(power > 0) {
            current = turn.firstAngle*-1;
            while (current <= degrees) {
                sleep(5);
                turn = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                current = turn.firstAngle * -1;
                if (opModeIsActive() == false) {
                    break;
                }
            }
        } else if(power <0) {
            current = turn.firstAngle;
            while(current <= degrees) {
                sleep(5);
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
}
