package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Blue: Park w/ pixel", group = "Competition")
public class parkWithPixelBlue extends LinearOpMode {

    public static Orientation angles;
    public static Acceleration gravity;

    BNO055IMU imu;


    public static void driveForward(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(speed);
        lB.setPower(speed);
        rF.setPower(speed * 1.2);
        rB.setPower(speed * 1.2);
    }

    public static void driveBackward(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(-speed);
        lB.setPower(-speed);
        rF.setPower(-speed);
        rB.setPower(-speed);
    }

    public static void turnLeft(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(-speed);
        lB.setPower(-speed);
        rF.setPower(speed);
        rB.setPower(speed);

    }

    public static void turnRight(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed)  {
        lF.setPower(speed);
        lB.setPower(speed);
        rF.setPower(-speed);
        rB.setPower(-speed);
    }

    public static void strafeLeft(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed)  {
        lF.setPower(-speed);
        lB.setPower(speed);
        rF.setPower(speed);
        rB.setPower(-speed);
    }

    public static void strafeRight(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed)  {
        lF.setPower(speed);
        lB.setPower(-speed);
        rF.setPower(-speed);
        rB.setPower(speed);
    }

    public static void driveForwardLeft(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(speed);
        lB.setPower(speed);
        rF.setPower(0);
        rB.setPower(0);
    }

    public static void driveForwardRight(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(speed);
        rB.setPower(speed);

    }

    public static void driveBackwardLeft(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed) {
        lF.setPower(-speed);
        lB.setPower(-speed);
        rF.setPower(0);
        rB.setPower(0);

    }

    public static void driveBackwardRight(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB, double speed)  {
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(-speed);
        rB.setPower(-speed);

    }

    public static void stop(DcMotor lF, DcMotor lB, DcMotor rF, DcMotor rB) {
        lF.setPower(0);
        lB.setPower(0);
        rF.setPower(0);
        rB.setPower(0);
    }




    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor lF = hardwareMap.dcMotor.get("front_left");
        DcMotor lB = hardwareMap.dcMotor.get("back_left");
        DcMotor rF = hardwareMap.dcMotor.get("front_right");
        DcMotor rB = hardwareMap.dcMotor.get("back_right");
        DcMotor extender = hardwareMap.dcMotor.get("steve");
        DcMotor rotator = hardwareMap.dcMotor.get("bob");

        Servo clawLeft = hardwareMap.servo.get("leftclaw");
        Servo clawRight = hardwareMap.servo.get("rightclaw");



        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests

        rF.setDirection(DcMotor.Direction.REVERSE);
        rB.setDirection(DcMotor.Direction.REVERSE);

        // Set motors to brake when not in use
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawRight.setPosition(0.035);
        clawLeft.setPosition(0);

        waitForStart();

        if (isStopRequested()) return;



        sleep(200);

        strafeLeft(lF, lB, rF, rB, -0.5);

        sleep(2100);

        return;
    }
}
