package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot2024 {
    Servo gripper;
    Servo gripperTurn;
    DcMotorEx Shoulder_Motor;

    DcMotorEx Elbow_Motor;
    double gripper_close_position = 0.988;
    double gripper_open_position = 0.163;
    double gripper_turn_open = 0.163;
    double gripper_turn_close = 0.988;
    double drivePowerModifier = 0.5;
    double shoulderModifier = 0.25;;
    double elbowModifier = 0.25;

    DcMotorEx leftFront;
    DcMotorEx rightFront;
    DcMotorEx leftBack;
    DcMotorEx rightBack;
    private double strafePower;
    private double forwardPower;
    private double turnPower;
    private double strafeVelocity;
    private double forwardVelocity;
    private double turnVelocity;

    private ElapsedTime runtime = new ElapsedTime();

    public Robot2024(Servo gripper, DcMotor Shoulder_Motor) {
        this.gripper = gripper;
    }

    public Robot2024(HardwareMap hardwareMap) {
        Shoulder_Motor = hardwareMap.get(DcMotorEx.class, "Shoulder_Motor");
        Shoulder_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

//        Shoulder_Motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(30.0, 0.0, 0.0, 0.0));
        Shoulder_Motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(30.0, 0.0, 0.0, 0.0));
        Shoulder_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        Elbow_Motor = hardwareMap.get(DcMotorEx.class, "Elbow_Motor");
        Elbow_Motor.setDirection(DcMotorSimple.Direction.FORWARD);

//        Elbow_Motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
//                new PIDFCoefficients(30.0, 0.0, 0.0, 0.0));
        Elbow_Motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(30.0, 0.0, 0.0, 0.0));
        Elbow_Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        
        this.gripper = hardwareMap.servo.get("Gripper_Servo");
        this.gripperTurn = hardwareMap.servo.get("Gripper_Turn");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        rightBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(20.0, 0.0, 0.0, 0.0));
        rightFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(20.0, 0.0, 0.0, 0.0));
        leftBack.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(20.0, 0.0, 0.0, 0.0));
        leftFront.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(20.0, 0.0, 0.0, 0.0));
    }

    public void closeGripper(){
        gripper.setPosition(this.gripper_close_position);
    }
    public void openGripper(){
        gripper.setPosition(this.gripper_open_position);
    }
    public void TurnGripperOpen(){
        gripperTurn.setPosition(this.gripper_turn_open);
    }
    public void TurnGripperClose(){
        gripperTurn.setPosition(this.gripper_turn_close);
    }

    public void setShoulderPower(double powerLevel){
//        Shoulder_Motor.setPower(powerLevel * this.shoulderModifier);
        Shoulder_Motor.setVelocity(powerLevel * this.shoulderModifier);
    }

    public void setElbowPower(double powerLevel) {
//        Elbow_Motor.setPower(powerLevel * this.elbowModifier);
        Elbow_Motor.setVelocity(powerLevel * this.elbowModifier);
    }

    public void setStrafePower(double powerLevel) {
        strafePower = powerLevel;
        this.setDrivePowers();
    }
    public void setStrafeRightPower(double powerLevel) {
        strafePower = powerLevel;
        this.setDrivePowers();
    }
    public void setStrafeLeftPower(double powerLevel) {
        strafePower = 0 - powerLevel;
        this.setDrivePowers();
    }

    public void timedMove(LinearOpMode owner,
                          double duration,
                          double strafePower,
                          double turnPower,
                          double forwardPower) {

        this.setForwardPower(forwardPower);
        this.setStrafePower(strafePower);
        this.setTurnPower(turnPower);

        runtime.reset();
        while (owner.opModeIsActive() && (runtime.seconds() < duration)) {
        }
        this.stopMoving();
    }

    public void timedVelocityMove(LinearOpMode owner,
                          double duration,
                          double strafeVelocity,
                          double turnVelocity,
                          double forwardVelocity) {

        this.setForwardVelocity(forwardVelocity);
        this.setStrafeVelocity(strafeVelocity);
        this.setTurnVelocity(turnVelocity);

        runtime.reset();
        while (owner.opModeIsActive() && (runtime.seconds() < duration)) {
        }
        this.stopMoving();
    }

    public void stopMoving() {
        this.setStrafePower(0);
        this.setForwardPower(0);
        this.setTurnPower(0);
        this.setDrivePowers();
    }

    public void setForwardPower(double powerLevel) {
        forwardPower = powerLevel;
        this.setDrivePowers();
    }

    public void setTurnPower(double powerLevel) {
        turnPower = powerLevel;
        this.setDrivePowers();
    }

    private void setDrivePowers() {
        rightFront.setPower((-strafePower - forwardPower + turnPower) * drivePowerModifier);
        leftFront.setPower((strafePower - forwardPower - turnPower) * drivePowerModifier);
        leftBack.setPower((-strafePower - forwardPower - turnPower) * drivePowerModifier);
        rightBack.setPower((strafePower - forwardPower + turnPower) * drivePowerModifier);
    }

    public void setStrafeVelocity(double velocity) {
        strafePower = velocity;
        this.setDriveVelocities();
    }

    public void setForwardVelocity(double velocity) {
        forwardPower = velocity;
        this.setDriveVelocities();
    }

    public void setTurnVelocity(double velocity) {
        turnPower = velocity;
        this.setDriveVelocities();
    }

    private void setDriveVelocities() {
        double rightFrontVelocityModifier = 1.0;
        double rightBackVelocityModifier = 1.0;
        double leftFrontVelocityModifier = 1.5;
        double leftBackVelocityModifier = 1.0;
        rightFront.setVelocity((-strafeVelocity - forwardVelocity + turnVelocity) * rightFrontVelocityModifier);
        leftFront.setVelocity((strafePower - forwardPower - turnPower) * leftFrontVelocityModifier);
        leftBack.setVelocity((-strafePower - forwardPower - turnPower) * leftBackVelocityModifier);
        rightBack.setVelocity((strafePower - forwardPower + turnPower) * rightBackVelocityModifier);
    }

    enum Motors {
        rightFront,
        leftFront,
        rightBack,
        leftBack,
        shoulder,
        elbow
    }

    DcMotorEx getMotor(Motors motor) {
        DcMotorEx motor_to_update = this.rightFront;
        if (motor == Motors.rightFront) {
            motor_to_update = this.rightFront;
        }
        else if (motor == Motors.leftFront) {
            motor_to_update = this.leftFront;
        }
        else if (motor == Motors.rightBack) {
            motor_to_update = this.rightBack;
        }
        else if (motor == Motors.leftBack) {
            motor_to_update = this.leftBack;
        }
        return motor_to_update;
    }

    void setPid(Motors motor, PIDFCoefficients coefficients) {
        DcMotorEx motor_to_update = this.rightFront;
        if (motor == Motors.rightFront) {
            motor_to_update = this.rightFront;
        }
        else if (motor == Motors.leftFront) {
            motor_to_update = this.leftFront;
        }
        else if (motor == Motors.rightBack) {
            motor_to_update = this.rightBack;
        }
        else if (motor == Motors.leftBack) {
            motor_to_update = this.leftBack;
        }

        motor_to_update.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coefficients);
    }
}
