package org.firstinspires.ftc.teamcode.auto;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Generic 2 Park", group = "Competition")
public class generic2Park extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//        Servo clawLeft = hardwareMap.servo.get("leftclaw");
//        Servo clawRight = hardwareMap.servo.get("rightclaw");
//        DcMotor arm = hardwareMap.dcMotor.get("rotator");
//        DcMotor extender = hardwareMap.dcMotor.get("extender");

//        clawRight.setPosition(0);
//        clawLeft.setPosition(0.035);

        BasicPID arm_controller = new BasicPID(new PIDCoefficients(0.05, 0, 0));
        double arm_target = 1;

        waitForStart();
        if (isStopRequested()) return;
//        clawRight.setPosition(0.035);
//        clawLeft.setPosition(0);

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-12.43, 61.25, Math.toRadians(267.75)))
                .lineTo(new Vector2d(-63.01, 60.37))
                .build();
        drive.setPoseEstimate(trajectory0.start());
        drive.followTrajectorySequence(trajectory0);
        return;
    }
}