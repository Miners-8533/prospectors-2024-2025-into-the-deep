package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(group="Robot")
public class AutonNetZone_Velocity extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        Robot2024 robot2024 = new Robot2024(hardwareMap);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();


        while (opModeIsActive()) {

            double rightFrontVelocityModifier = 1.0;
            double rightBackVelocityModifier = 1.0;
            double leftFrontVelocityModifier = 1.07;
            double leftBackVelocityModifier = 1.0;

//            robot2024.rightFront.setVelocity(-1000.0*rightFrontVelocityModifier);
            robot2024.rightBack.setVelocity(-1000.0*rightBackVelocityModifier);
//            robot2024.leftFront.setVelocity(-1000.0*leftFrontVelocityModifier);
            robot2024.leftBack.setVelocity(-1000.0*leftBackVelocityModifier);
        }
//        robot2024.timedVelocityMove(this, 1.25, 1.0, 0, 0.0);
//        robot2024.timedVelocityMove(this, 1.25,  0.6,  0, 0.75);
//        robot2024.timedVelocityMove(this, 1.9,  0.0,  -0.75, 0.0);
//        robot2024.timedVelocityMove(this, 1.35,  0.0,  0.0, 0.75);
//        robot2024.timedVelocityMove(this, 1.0,  0.0,  0.3, 0.0);
//        robot2024.timedVelocityMove(this, 0.5,  0.75,  0.0, -0.50);
//        robot2024.timedVelocityMove(this, 0.5,  0.75,  0.0, 0.0);
//        robot2024.timedVelocityMove(this, 1.5,  -0.75,  0.0, 0.0);
//        robot2024.timedVelocityMove(this, 1.5,  0.0,  0.0, -0.75);
//        robot2024.timedVelocityMove(this, 1.0,  0.75,  0.0, 0.0);
//        robot2024.timedVelocityMove(this, 2.0,  0.0,  0.0, 0.75);
//        robot2024.timedVelocityMove(this, 2.0,  0.0,  0.0, -0.80);
//        robot2024.timedVelocityMove(this, 0.5,  0.75,  0.0, 0.0);
//        robot2024.timedVelocityMove(this, 2.0,  0.0,  0.0, 0.75);
//        robot2024.timedVelocityMove(this, 2.0,  0.0,  0.0, -0.75);
    }
}
