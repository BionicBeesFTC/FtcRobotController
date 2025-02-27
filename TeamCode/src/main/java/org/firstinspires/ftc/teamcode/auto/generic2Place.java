package org.firstinspires.ftc.teamcode.auto;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Generic 2 Place & Park", group = "Competition")
public class generic2Place extends LinearOpMode {

    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        DcMotor extender1 = hardwareMap.dcMotor.get("extender1");
        DcMotor extender2 = hardwareMap.dcMotor.get("extender2");
        DcMotor wrist = hardwareMap.dcMotor.get("rotator");
        Servo armjoint = hardwareMap.servo.get("armjoint");
        CRServo leftturn = hardwareMap.crservo.get("leftturn");
        CRServo rightturn = hardwareMap.crservo.get("rightturn");



        // Set the motors to use encoders
        extender1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wrist.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wrist.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set up the PID controller
        PIDCoefficients coeffs = new PIDCoefficients(0.005, 0.0, 0.0);
        BasicPID extController = new BasicPID(coeffs);
        PIDCoefficients wristCoeffs = new PIDCoefficients(0.01, 0.00, 0.00);
        BasicPID wristController = new BasicPID(wristCoeffs);

        // Target position for the extender motors (modify as needed)
        int targetPosition = 3800;

        double pidOutput = 0.0;
        int position1 = 0;
        int position2 = 0;
        int averagePosition = 0;
        telemetry.addData("pos1", position1);
        armjoint.setPosition(0);


        waitForStart();
        if (isStopRequested()) return;

        TrajectorySequence trajectory0 = drive.trajectorySequenceBuilder(new Pose2d(-11.99, -60.10, Math.toRadians(90.81)))
                .splineTo(new Vector2d(-35.96, -48.38), Math.toRadians(194.59))
                .splineTo(new Vector2d(-57.20, -48.56), Math.toRadians(222.27))
                .build();
        drive.setPoseEstimate(trajectory0.start());
        drive.followTrajectorySequence(trajectory0);

        boolean reachedFirst = false;
        boolean reachedSecond = false;
        boolean placed = false;

        int iter_count = 0;
        int iter2 = 0;



        double wristTarget = 0.0;
        while (opModeIsActive()) {
            if (isStopRequested()) {
                extender1.setPower(0);
                extender2.setPower(0);
                wrist.setPower(0);
                break;
            }

            position1 = -extender1.getCurrentPosition();
            position2 = extender2.getCurrentPosition();

            averagePosition = (position1 + position2) / 2;

            pidOutput = extController.calculate(averagePosition, targetPosition);

            extender1.setPower(pidOutput);
            extender2.setPower(-pidOutput);

            double wristOutput = wristController.calculate(wrist.getCurrentPosition(), wristTarget);
            wrist.setPower(-wristOutput);
            telemetry.addData("wrist", wrist.getCurrentPosition());
            telemetry.addData("wristpid", wristOutput);
            telemetry.update();

            if (Math.abs(averagePosition - targetPosition) < 100 || averagePosition > targetPosition) {
                reachedFirst = true;
            }

            if (reachedFirst && !placed) {
                wristTarget = 550;
                if (wrist.getCurrentPosition() > wristTarget) {
                    reachedSecond = true;
                }

                if (reachedSecond) {
                    iter_count++;
                    if (iter_count > 50) {
                        armjoint.setPosition(-0.050);
                        leftturn.setPower(-1);
                        rightturn.setPower(1);
                    }
                    if (iter_count > 90) {
                        placed = true;
                    }
                }
            }

            if (placed) {
                iter2++;
                if (iter2 > 30) {
                    wristTarget = 5;
                }
                if (iter2 > 40) {
                    targetPosition = 3;
                }
                if (iter2 > 400) {
                    System.out.println("made it");
                    extender1.setPower(0);
                    extender2.setPower(0); 
                    wrist.setPower(0);
                    leftturn.setPower(0);
                    rightturn.setPower(0);
                    break;
                }
            }
            //sleep(10);
        }

      //  TrajectorySequence trajectory1 = drive.trajectorySequenceBuilder(new Pose2d(-58.60, -57.19, Math.toRadians(228.30)))
        //        .splineTo(new Vector2d(-43.62, -47.68), Math.toRadians(-2.62))
          //      .splineTo(new Vector2d(25.12, -46.09), Math.toRadians(-34.90))
            //    .splineTo(new Vector2d(46.27, -61.25), Math.toRadians(270.00))
              //  .build();
      //  drive.setPoseEstimate(trajectory1.start());
        //  drive.followTrajectorySequence(trajectory1);
    }
}
