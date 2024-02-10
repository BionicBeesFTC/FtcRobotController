package org.firstinspires.ftc.teamcode;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Blue 1: Place Pixel", group = "Competition")
public class pixelOnBoardCloseBlue extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo clawLeft = hardwareMap.servo.get("leftclaw");
        Servo clawRight = hardwareMap.servo.get("rightclaw");
        DcMotor arm = hardwareMap.dcMotor.get("rotator");
        DcMotor extender = hardwareMap.dcMotor.get("extender");

        clawRight.setPosition(0);
        clawLeft.setPosition(0.035);

        BasicPID arm_controller = new BasicPID(new PIDCoefficients(0.05, 0, 0));
        double arm_target = 1;


        clawRight.setPosition(0);
        clawLeft.setPosition(0.035);
        waitForStart();
        if (isStopRequested()) return;
        clawRight.setPosition(0.035);
        clawLeft.setPosition(0);

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(12.07, 60.90, Math.toRadians(-88.88)))
                .waitSeconds(1)
                .splineTo(new Vector2d(48.73, 34.11), Math.toRadians(0.00))
                .waitSeconds(2)
                .splineTo(new Vector2d(42.57, 58.96), Math.toRadians(1.97))
                .splineTo(new Vector2d(62.31, 59.13), Math.toRadians(0.00))
                .build();
        drive.setPoseEstimate(untitled0.start());
        drive.setPoseEstimate(untitled0.start());
        drive.followTrajectorySequence(untitled0);
        return;
    }
}