package org.firstinspires.ftc.teamcode.teleop;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
        DcMotor extender = hardwareMap.dcMotor.get("extender");
        DcMotor rotator = hardwareMap.dcMotor.get("rotator");

        Servo clawLeft = hardwareMap.servo.get("leftclaw");
        Servo clawRight = hardwareMap.servo.get("rightclaw");
        CRServo Drone = hardwareMap.crservo.get("Drone");



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

        Drone.setPower(0);

        BasicPID arm_controller = new BasicPID(new PIDCoefficients(KP, KI, KD));

        double arm_target = 1;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


            double rx = -gamepad1.right_stick_x; // Remember, this is reversed!
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double y = gamepad1.left_stick_y;


            double frontLeftPower = y + x + rx;
            double backLeftPower = y - x + rx;
            double frontRightPower = y - x - rx;
            double backRightPower = y + x - rx;

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
                telemetry.update();
            } else if (gamepad1.left_bumper) {
                lF.setPower(frontLeftPower * 0.75);
                lB.setPower(backLeftPower * 0.75);
                rF.setPower(frontRightPower * 0.75);
                rB.setPower(backRightPower * 0.75);
                telemetry.addLine("Speed 3/4");
                telemetry.update();
            } else {
                lF.setPower(frontLeftPower);
                lB.setPower(backLeftPower);
                rF.setPower(frontRightPower);
                rB.setPower(backRightPower);
                telemetry.addLine("Speed full");
                telemetry.update();
            }

            if(gamepad2.dpad_up){
                extender.setPower(1);
            } else if(gamepad2.dpad_down){
                extender.setPower(-1);
            } else {
                extender.setPower(0);
            }

            if(gamepad2.right_trigger > 0.5) {
                // close
                clawRight.setPosition(0.035);
                clawLeft.setPosition(0);
            }

            if(gamepad2.left_trigger > 0.5) {
                // open
                clawRight.setPosition(0);
                clawLeft.setPosition(0.035);
            }

            if(gamepad2.x) {
                //paper plane launcher
                Drone.setPower(1);
            }
            else {
                Drone.setPower(0);
            }

            // slightly above ground: 18
            // lifted to board: 120
            // set on ground: 1-5
            if (gamepad2.right_bumper) {
                arm_target += -gamepad2.right_stick_y;
            } else {
                arm_target += -gamepad2.right_stick_y * 1.45;
            }

            if (arm_target > 228) {
                arm_target = 228;
            } else if (arm_target < 2) {
                arm_target = 2;
            }

            rotator.setPower(arm_controller.calculate(arm_target + 6, rotator.getCurrentPosition()));
            //TelemetryPacket packet = new TelemetryPacket();
            //packet.put("Arm Position", rotator.getCurrentPosition());
            //packet.put("Arm Target", arm_target);
            //FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }
}
