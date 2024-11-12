package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group="Robot")
public class AutonNetZone extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        Robot2024 robot2024 = new Robot2024(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();


        robot2024.timedMove(this, 1.25, -0.75, 0, 0.0);
        robot2024.timedMove(this, 1.25,  0.6,  0, 0.75);
        robot2024.timedMove(this, 1.9,  0.0,  -0.75, 0.0);
        robot2024.timedMove(this, 1.35,  0.0,  0.0, 0.75);
        robot2024.timedMove(this, 1.0,  0.0,  0.3, 0.0);
        robot2024.timedMove(this, 0.5,  0.75,  0.0, -0.50);
        robot2024.timedMove(this, 0.5,  0.75,  0.0, 0.0);
        robot2024.timedMove(this, 1.5,  -0.75,  0.0, 0.0);
        robot2024.timedMove(this, 1.5,  0.0,  0.0, -0.75);
        robot2024.timedMove(this, 1.0,  0.75,  0.0, 0.0);
        robot2024.timedMove(this, 2.0,  0.0,  0.0, 0.75);
        robot2024.timedMove(this, 2.0,  0.0,  0.0, -0.80);
        robot2024.timedMove(this, 0.5,  0.75,  0.0, 0.0);
        robot2024.timedMove(this, 2.0,  0.0,  0.0, 0.75);
        robot2024.timedMove(this, 2.0,  0.0,  0.0, -0.75);
    }
}
