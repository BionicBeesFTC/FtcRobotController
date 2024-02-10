package org.firstinspires.ftc.teamcode.auto.red;
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

@Autonomous(name = "Red 2: Park w/ Pixel In", group = "Competition")
public class red2ParkIn extends LinearOpMode {

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

        waitForStart();
        if (isStopRequested()) return;
        clawRight.setPosition(0.035);
        clawLeft.setPosition(0);

        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(-36.75, -60.37, Math.toRadians(92.20)))
                .waitSeconds(1)
                .splineTo(new Vector2d(-30.58, 9.25), Math.toRadians(352.41))
                .splineTo(new Vector2d(18.59, 5.73), Math.toRadians(327.96))
                .splineTo(new Vector2d(24.41, -18.59), Math.toRadians(275.87))
                .splineTo(new Vector2d(34.63, -42.04), Math.toRadians(327.85))
                .splineTo(new Vector2d(72.00, -53.44), Math.toRadians(364.40))
                .build();
        drive.setPoseEstimate(untitled0.start());
        drive.followTrajectorySequence(untitled0);
        return;
    }
}