package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele2")
public class Tele2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();
        /*
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (opModeIsActive()) {
            if (gamepad1.left_stick_y > 0.2) {
                motor.setPower(gamepad2.left_stick_y * 0.3);
            }
            else {
                motor.setPower(0);
            }
            telemetry.addData("Extension motor power", motor.getPower());
            telemetry.addData("Extension encoder: ", motor.getCurrentPosition());
            telemetry.update();
        }*/
        ServoImplEx servo = hardwareMap.get(ServoImplEx.class, "CollectHingeServo");
        servo.setPwmRange(new PwmControl.PwmRange(Collector.HINGE_DOWN_TICK, Collector.HINGE_UP_TICK));
        while(opModeIsActive()) {
            if(gamepad1.a) {
                servo.setPosition(0.01);
            }
            else if(gamepad1.b) {
                servo.setPosition(0.99);
            }
            telemetry.addData("a", gamepad1.a);
            telemetry.addData("b",gamepad1.b);
            telemetry.addData("servo range", servo.getPwmRange());
            telemetry.addData("servo position", servo.getPosition());
            telemetry.update();
        }
    }
}
