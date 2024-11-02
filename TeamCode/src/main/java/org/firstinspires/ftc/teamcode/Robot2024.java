package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Robot2024 {
    Servo gripper;
    Servo gripperTurn;
    DcMotor Shoulder_Motor;

    DcMotor Elbow_Motor;
    double gripper_close_position = 0.988;
    double gripper_open_position = 0.163;
    double gripper_turn_open = 0.163;
    double gripper_turn_close = 0.988;
    double drivePowerModifier = 0.5;
    double shoulderModifier = 0.25;;
    double elbowModifier = 0.25;

    private DcMotor leftFront;
    private DcMotor rightFront;
    private DcMotor leftBack;
    private DcMotor rightBack;
    private double strafePower;
    private double forwardPower;
    private double turnPower;

    private ElapsedTime runtime = new ElapsedTime();

    public Robot2024(Servo gripper, DcMotor Shoulder_Motor) {
        this.gripper = gripper;
        this.Shoulder_Motor = Shoulder_Motor;
    }

    public Robot2024(HardwareMap hardwareMap) {
        this.Shoulder_Motor = hardwareMap.dcMotor.get("Shoulder_Motor");
        this.Shoulder_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.Shoulder_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.Elbow_Motor = hardwareMap.dcMotor.get("Elbow_Motor");
        this.Elbow_Motor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.Elbow_Motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.gripper = hardwareMap.servo.get("Gripper_Servo");
        this.gripperTurn = hardwareMap.servo.get("Gripper_Turn");
        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack = hardwareMap.dcMotor.get("leftBack");
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack = hardwareMap.dcMotor.get("rightBack");
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
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
        Shoulder_Motor.setPower(powerLevel * this.shoulderModifier);
    }

    public void setElbowPower(double powerLevel) {
        Elbow_Motor.setPower(powerLevel * this.elbowModifier);
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

}
