package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue: Park w/ pixel", group = "Competition")
public class parkWithPixelBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo clawLeft = hardwareMap.servo.get("leftclaw");
        Servo clawRight = hardwareMap.servo.get("rightclaw");
        DcMotor arm = hardwareMap.dcMotor.get("rotator");
        clawRight.setPosition(0);
        clawLeft.setPosition(0.035);

        waitForStart();
        if (isStopRequested()) return;


        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(11.46, 60.64, Math.toRadians(-87.88)))
                .UNSTABLE_addTemporalMarkerOffset(1.89,() -> {arm.setPower(0.5);})
                .UNSTABLE_addTemporalMarkerOffset(2.0,() -> {clawRight.setPosition(0.035);clawLeft.setPosition(0);})
                .UNSTABLE_addTemporalMarkerOffset(5.41,() -> {arm.setPower(0);})
                .splineTo(new Vector2d(44.40, 28.44), Math.toRadians(0.00))
                .splineTo(new Vector2d(28.97, 47.22), Math.toRadians(120.58))
                .splineTo(new Vector2d(58.40, 59.52), Math.toRadians(4.09))
                .build();
        drive.setPoseEstimate(untitled0.start());



        drive.followTrajectorySequence(untitled0);

        return;
    }
}