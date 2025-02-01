package org.firstinspires.ftc.teamcode.auto;

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

@Autonomous(name = "Generic 2 Place & Park", group = "Competition")
public class generic2Place extends LinearOpMode {
    private enum RobotState {
        INITIALIZING,
        DRIVING_TO_POSITION,
        EXTENDING,
        ROTATING_WRIST,
        RELEASING,
        RETRACTING,
        LOWERING,
        PARKING,
        COMPLETED
    }

    private static final int EXTENDER_TARGET = 3750;
    private static final int WRIST_EXTENDED = 600;
    private static final int WRIST_RETRACTED = 5;
    private static final double CLAW_OPEN = 0.040;
    private static final double CLAW_CLOSED = 0.0;
    
    private RobotState currentState = RobotState.INITIALIZING;
    private SampleMecanumDrive drive;
    private DcMotor extender1, extender2, wrist;
    private Servo clawLeft;
    private BasicPID extController, wristController;

    @Override
    public void runOpMode() {
        initializeHardware();
        
        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive() && currentState != RobotState.COMPLETED) {
            switch (currentState) {
                case INITIALIZING:
                    executeTrajectory0();
                    currentState = RobotState.EXTENDING;
                    break;

                case EXTENDING:
                    if (moveExtenderToPosition(EXTENDER_TARGET) && moveWristToPosition(WRIST_RETRACTED)) {
                        currentState = RobotState.ROTATING_WRIST;
                    }
                    break;

                case ROTATING_WRIST:
                    if (moveWristToPosition(WRIST_EXTENDED) && moveExtenderToPosition(EXTENDER_TARGET)) {
                        currentState = RobotState.RELEASING;
                    }
                    break;

                case RELEASING:
                    clawLeft.setPosition(CLAW_OPEN);
                    sleep(100);
                    currentState = RobotState.RETRACTING;
                    break;

                case RETRACTING:
                    if (moveExtenderToPosition(EXTENDER_TARGET) && moveWristToPosition(WRIST_RETRACTED)) {
                        currentState = RobotState.LOWERING;
                    }
                    break;

                case LOWERING:
                    if (moveExtenderToPosition(20) && moveWristToPosition(WRIST_RETRACTED)) {
                        currentState = RobotState.PARKING;
                    }
                    break;
            
                case PARKING:
                    executeTrajectory1();
                    currentState = RobotState.COMPLETED;
                    break;
            }
            
            updateTelemetry();
            sleep(10);
        }
    }

    private void initializeHardware() {
        drive = new SampleMecanumDrive(hardwareMap);
        extender1 = hardwareMap.dcMotor.get("extender1");
        extender2 = hardwareMap.dcMotor.get("extender2");
        wrist = hardwareMap.dcMotor.get("rotator");
        clawLeft = hardwareMap.servo.get("leftclaw");

        initializeMotors();
        initializePIDControllers();
        clawLeft.setPosition(CLAW_CLOSED);
    }

    private void initializeMotors() {
        DcMotor[] motors = {extender1, extender2, wrist};
        for (DcMotor motor : motors) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void initializePIDControllers() {
        extController = new BasicPID(new PIDCoefficients(0.005, 0.0, 0.0));
        wristController = new BasicPID(new PIDCoefficients(0.01, 0.00, 0.00));
    }

    private boolean moveExtenderToPosition(int target) {
        int position1 = -extender1.getCurrentPosition();
        int position2 = extender2.getCurrentPosition();
        int averagePosition = (position1 + position2) / 2;
        
        double pidOutput = extController.calculate(averagePosition, target);
        extender1.setPower(pidOutput);
        extender2.setPower(-pidOutput);
        
        return Math.abs(averagePosition - target) < 100;
    }

    private boolean moveWristToPosition(int target) {
        double wristOutput = wristController.calculate(wrist.getCurrentPosition(), target);
        wrist.setPower(-wristOutput);
        return Math.abs(wrist.getCurrentPosition() - target) < 50;
    }

    private void executeTrajectory0() {
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(-11.99, -60.10, Math.toRadians(90.81)))
                .splineTo(new Vector2d(-35.96, -48.38), Math.toRadians(194.59))
                .splineTo(new Vector2d(-57.20, -48.56), Math.toRadians(222.27))
                .build();
        drive.setPoseEstimate(trajectory.start());
        drive.followTrajectorySequence(trajectory);
    }

    private void executeTrajectory1() {
        TrajectorySequence trajectory = drive.trajectorySequenceBuilder(new Pose2d(-58.60, -57.19, Math.toRadians(228.30)))
                .splineTo(new Vector2d(-43.62, -47.68), Math.toRadians(-2.62))
                .splineTo(new Vector2d(25.12, -46.09), Math.toRadians(-34.90))
                .splineTo(new Vector2d(46.27, -61.25), Math.toRadians(270.00))
                .build();
        drive.setPoseEstimate(trajectory.start());
        drive.followTrajectorySequence(trajectory);
    }

    private void updateTelemetry() {
        telemetry.addData("State", currentState);
        telemetry.addData("Wrist Position", wrist.getCurrentPosition());
        telemetry.addData("Extender Position", (extender1.getCurrentPosition() + extender2.getCurrentPosition()) / 2);
        telemetry.update();
    }
}