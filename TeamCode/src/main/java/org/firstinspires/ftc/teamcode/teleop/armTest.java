package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class armTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        // Position of the arm when it's down
        int armPosition = 0;

        // Find a motor in the hardware map named "Arm Motor"
        DcMotor armMotor = hardwareMap.dcMotor.get("rotator");

        // Reset the motor encoder so that it reads zero ticks
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Sets the starting position of the arm to the down position
//        armMotor.setTargetPosition(armPosition);
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        while (opModeIsActive()) {
            // If the A button is pressed, raise the arm

            if (gamepad2.a && armMotor.getCurrentPosition() >0) {
//                armMotor.setTargetPosition(armPosition);
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(-0.5);
            }else if (gamepad2.b && armMotor.getCurrentPosition() <200) {
//                armPosition =armPosition +1;
//                armMotor.setTargetPosition(armPosition);
//                armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                armMotor.setPower(0.5);
            }else {
                //TODO: give power to keep at a certain position when not moving
            }

            // Get the current position of the armMotor
            double position = armMotor.getCurrentPosition();

            // Get the target position of the armMotor
            double desiredPosition = armMotor.getTargetPosition();

            // Show the position of the armMotor on telemetry
            telemetry.addData("Encoder Position", position);

            // Show the target position of the armMotor on telemetry
            telemetry.addData("Desired Position", desiredPosition);

            telemetry.update();
        }
    }
}
