package org.firstinspires.ftc.teamcode.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Config
@TeleOp(name = "Standard (Two-Driver)", group = "Competition")
public class standardTwoDriver extends LinearOpMode {

    public static Orientation angles;
    public static Acceleration gravity;

    public static double KP = 0.05;
    public static double KI = 0;
    public static double KD = 0;


    BNO055IMU imu;

    TouchSensor lim_1;


    public void initIMU(HardwareMap hwm) {
        imu = hwm.get(BNO055IMU.class, "imu 1");
        BNO055IMU.Parameters parameters1 = new BNO055IMU.Parameters();
        parameters1.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters1.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters1.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters1.loggingEnabled = true;
        parameters1.loggingTag = "IMU";
        parameters1.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters1);

        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }


    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor lF = hardwareMap.dcMotor.get("front_left");
        DcMotor lB = hardwareMap.dcMotor.get("back_left");
        DcMotor rF = hardwareMap.dcMotor.get("front_right");
        DcMotor rB = hardwareMap.dcMotor.get("back_right");
        DcMotor extender1 = hardwareMap.dcMotor.get("extender1");
        DcMotor extender2 = hardwareMap.dcMotor.get("extender2");
        DcMotor rotator = hardwareMap.dcMotor.get("rotator");

        lim_1 = hardwareMap.touchSensor.get("lim_1");

        PIDCoefficients coeffs = new PIDCoefficients(0.005, 0.0, 0.0);
        BasicPID extController = new BasicPID(coeffs);

        Servo armjoint = hardwareMap.servo.get("armjoint");
        armjoint.setPosition(0);
        CRServo leftturn = hardwareMap.crservo.get("leftturn");
        //CRServo rightturn = hardwareMap.crservo.get("rightturn");


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests

        rF.setDirection(DcMotor.Direction.REVERSE);
        rB.setDirection(DcMotor.Direction.REVERSE);

        extender1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set motors to brake when not in use
        lF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double pidOutput = 0.0;
        int position1 = 0;
        int position2 = 0;
        int averagePosition = 0;
        int extTargetPosition = 0;

        rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotator.setTargetPosition(0);
        rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        BasicPID arm_controller = new BasicPID(new PIDCoefficients(KP, KI, KD));
        int arm_target = 1;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            position1 = -extender1.getCurrentPosition();
            position2 = extender2.getCurrentPosition();

            averagePosition = (position1 + position2) / 2;

            pidOutput = extController.calculate(averagePosition, extTargetPosition);

            extender1.setPower(pidOutput);
            extender2.setPower(-pidOutput);

            double rx = -gamepad1.right_stick_x; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double y = gamepad1.left_stick_y;


            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

            int MAX_ARM_HEIGHT = 4300;
            //used to set the height of the extendor
            // Put powers in the range of -1 to 1 only if they aren't already
            // Not checking would cause us to always drive at full speed
            if (Math.abs(frontLeftPower) > 1 || Math.abs(backLeftPower) > 1 ||
                    Math.abs(frontRightPower) > 1 || Math.abs(backRightPower) > 1) {
                // Find the largest power
                double max = 0;
                max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
                max = Math.max(Math.abs(frontRightPower), max);
                max = Math.max(Math.abs(backRightPower), max);

                // Divide everything by max (it's positive so we don't need to worry
                // about signs)
                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;
            }
            if (gamepad1.right_bumper) {
                lF.setPower(frontLeftPower * 0.25);
                lB.setPower(backLeftPower * 0.25);
                rF.setPower(frontRightPower * 0.25);
                rB.setPower(backRightPower * 0.25);
                telemetry.addLine("Speed one quarter");
//                telemetry.update();
            } else if (gamepad1.left_bumper) {
                lF.setPower(frontLeftPower * 0.75);
                lB.setPower(backLeftPower * 0.75);
                rF.setPower(frontRightPower * 0.75);
                rB.setPower(backRightPower * 0.75);
                telemetry.addLine("Speed 3/4");
//                telemetry.update();
            } else {
                lF.setPower(frontLeftPower);
                lB.setPower(backLeftPower);
                rF.setPower(frontRightPower);
                rB.setPower(backRightPower);
                telemetry.addLine("Speed full");
//                telemetry.update();
            }

                if (gamepad2.dpad_up) {
                    extTargetPosition += 50;
                } else if (gamepad2.dpad_down) {
                    extTargetPosition -= 50;
                }

                if (extTargetPosition < 90) {
                    extTargetPosition = 90;
                } else if (extTargetPosition > 4300) {
                    extTargetPosition = 4300;
                }

                //TODO:Delete this code after fixing arm extention limit.

                telemetry.addLine("extender 1:" + -extender1.getCurrentPosition());
                telemetry.addLine("extender 2:" + extender2.getCurrentPosition());
                telemetry.addLine("extender avg:" + averagePosition);
                telemetry.addLine("target: " + extTargetPosition);

//            telemetry.update();

                if (!gamepad2.right_bumper && !gamepad2.left_bumper) {
                    leftturn.setPower(0);
                }



                if (gamepad2.left_bumper) {
                    leftturn.setPower(0.75);
                }

                if (gamepad2.right_bumper) {
                    leftturn.setPower(-0.75);
                }




                if (gamepad2.left_stick_y > 0) {
                    armjoint.setPosition(armjoint.getPosition()-0.001);
                }

                if (gamepad2.left_stick_y <0) {
                    armjoint.setPosition(armjoint.getPosition()+0.001);
                }

                if (gamepad1.a) {

                    rotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    rotator.setTargetPosition(0);
                    rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                int currentPos = rotator.getCurrentPosition();
                arm_target += gamepad2.right_stick_y * 10.0;
                if (!gamepad1.x) {
                    if (arm_target > 1380) {
                        arm_target = 1380;
                    } else if (arm_target < 0) {
                        arm_target = 0;
                    }
                }

                telemetry.addLine("lim " + lim_1.isPressed());

                if (lim_1.isPressed()) {
                    telemetry.addLine("LIMSWITCH");
                    extender1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extender2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    extender1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    extender2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                double pidPower = arm_controller.calculate(arm_target, currentPos);
                rotator.setTargetPosition(arm_target);
                rotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotator.setPower(-pidPower);//having it negative should make the +y JS go up and -y JS go down

                telemetry.addLine("Right Stick Y Value: " + gamepad2.right_stick_y);
                telemetry.addLine("Current Position: " + currentPos);
                telemetry.addLine("TEST: " + rotator.getCurrentPosition());
                telemetry.addLine("Target Position: " + arm_target);
                telemetry.addLine("PID Power: " + pidPower);
                telemetry.update();
            }
        }
    }
