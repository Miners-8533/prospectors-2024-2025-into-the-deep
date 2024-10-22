package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class GripperTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor Shoulder_Motor = hardwareMap.dcMotor.get("Shoulder_Motor");
        Shoulder_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        Servo Gripper = hardwareMap.servo.get("Gripper_Servo");
        Servo Gripper_Turn = hardwareMap.servo.get("Gripper_Turn");


        Robot2024 robot2024 = new Robot2024(hardwareMap);
        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {

            double modifier = 0.5;

            robot2024.setShoulderPower((gamepad2.left_stick_x - gamepad2.left_stick_y - gamepad2.right_stick_x) * modifier);



            if (gamepad2.b) {
                robot2024.closeGripper();
            } else if (gamepad2.a) {
               robot2024.openGripper();
            } else if (gamepad2.y) {
                robot2024.TurnGripperOpen();
            } else if (gamepad2.x) {
                robot2024.TurnGripperClose();
            }
        telemetry.addData("Servo Position", Gripper.getPosition());
        telemetry.addData("Status", "Running");
        telemetry.update();
        }

    }
}