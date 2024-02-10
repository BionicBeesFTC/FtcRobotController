package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

@Disabled
@TeleOp(name = "imu test", group = ".")
public class imuTest extends LinearOpMode {
    IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                DriveConstants.LOGO_FACING_DIR, DriveConstants.USB_FACING_DIR)));

        if (isStopRequested()){
            return;
        }

        waitForStart();
        while (opModeIsActive()) {
            AngularVelocity velocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
            telemetry.addData("x rot: ", velocity.zRotationRate);
            telemetry.addData("y rot: ", velocity.yRotationRate);
            telemetry.addData("z rot: ", velocity.zRotationRate);
            telemetry.addData("unit ", velocity.unit);
            telemetry.update();
        }
    }
}