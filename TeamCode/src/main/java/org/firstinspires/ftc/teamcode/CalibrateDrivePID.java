package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.HashMap;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "Test")
public class CalibrateDrivePID extends LinearOpMode {


    double debounceDelay = 0.1;
    double debounceTime = 0.0;

    HashMap<Robot2024.Motors, Double> pidHashmap = new HashMap<Robot2024.Motors, Double>();
    Robot2024.Motors motor_to_calibrate = Robot2024.Motors.rightFront;
    @Override
    public void runOpMode() throws InterruptedException {

        Robot2024 robot2024 = new Robot2024(hardwareMap);

        this.pidHashmap.put(Robot2024.Motors.rightFront, 1.0);
        this.pidHashmap.put(Robot2024.Motors.rightBack, 1.0);
        this.pidHashmap.put(Robot2024.Motors.leftFront, 1.0);
        this.pidHashmap.put(Robot2024.Motors.leftBack, 1.0);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad2.dpad_up) {
                this.motor_to_calibrate = Robot2024.Motors.rightFront;
            } else if (gamepad2.dpad_right) {
                this.motor_to_calibrate = Robot2024.Motors.rightBack;
            } else if (gamepad2.dpad_down) {
                this.motor_to_calibrate = Robot2024.Motors.leftBack;
            } else if (gamepad2.dpad_left) {
                this.motor_to_calibrate = Robot2024.Motors.leftFront;
            }


            if (this.time - this.debounceTime > this.debounceDelay) {

                if (gamepad2.y) {
                    this.pidHashmap.put(this.motor_to_calibrate, this.pidHashmap.get(this.motor_to_calibrate)*10);
                    robot2024.setPid(this.motor_to_calibrate,
                            new PIDFCoefficients(this.pidHashmap.get(this.motor_to_calibrate), 0.0, 0.0, 0.0));
                } else if (gamepad2.a){
                    this.pidHashmap.put(this.motor_to_calibrate, this.pidHashmap.get(this.motor_to_calibrate)/10);
                    robot2024.setPid(this.motor_to_calibrate,
                            new PIDFCoefficients(this.pidHashmap.get(this.motor_to_calibrate), 0.0, 0.0, 0.0));
                }

                this.debounceTime = this.time;
            }

            if (gamepad2.x) {
                robot2024.getMotor(this.motor_to_calibrate).setVelocity(1000.0);
            } else {
                robot2024.getMotor(this.motor_to_calibrate).setVelocity(0.0);
            }

            telemetry.addLine("Setting PID for " + this.motor_to_calibrate);
            this.printPidReport();
            telemetry.update();
        }

    }

    void printPidReport(){
        telemetry.addLine();
        telemetry.addLine("Current PID Values");
        telemetry.addData("rightFront", this.pidHashmap.get(Robot2024.Motors.rightFront));
        telemetry.addData("rightBack", this.pidHashmap.get(Robot2024.Motors.rightBack));
        telemetry.addData("leftFront", this.pidHashmap.get(Robot2024.Motors.leftFront));
        telemetry.addData("leftBack", this.pidHashmap.get(Robot2024.Motors.leftBack));
    }
}