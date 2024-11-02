package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        Robot2024 robot2024 = new Robot2024(hardwareMap);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            robot2024.setTurnPower(gamepad1.right_stick_x);
            robot2024.setForwardPower(gamepad1.left_stick_y);
            robot2024.setStrafePower(gamepad1.left_stick_x);

            robot2024.setShoulderPower(gamepad2.left_stick_y);
            robot2024.setElbowPower(gamepad2.right_stick_y);

            if (gamepad2.b) {
                robot2024.closeGripper();
            } else if (gamepad2.a) {
                robot2024.openGripper();
            }  else if (gamepad2.y) {
                robot2024.TurnGripperClose();
            } else if (gamepad2.x) {
                robot2024.TurnGripperOpen();
            }

//        telemetry.addData("Servo Position", servoTest.getPosition());
//        telemetry.addData("Target Power", tgtPower);
//        telemetry.addData("Status", "Running");
//        telemetry.update();
        }

    }
}