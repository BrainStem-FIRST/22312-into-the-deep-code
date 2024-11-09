package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Tele2")
public class Tele2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "LiftMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ServoImplEx armServo = hardwareMap.get(ServoImplEx.class, "LiftArmServo");
        armServo.setPwmRange(new PwmControl.PwmRange(Arm.MIN_TICK, Arm.MAX_TICK));

        ServoImplEx grabberServo = hardwareMap.get(ServoImplEx.class, "LiftGrabServo");
        grabberServo.setPwmRange(new PwmControl.PwmRange(Grabber.MIN_TICK, Grabber.MAX_TICK));
        while(opModeIsActive()) {
            if(gamepad1.a)
                grabberServo.setPosition(0.99);
            else if(gamepad1.b)
                grabberServo.setPosition(0.01);

            if(gamepad1.dpad_left)
                armServo.setPosition(Arm.DOWN_POS);
            else if(gamepad1.dpad_right)
                armServo.setPosition(Arm.RIGHT_POS);

            if(gamepad1.dpad_down)
                Subsystem.setMotorPower(motor, -0.3);
            else if(gamepad1.dpad_up)
                Subsystem.setMotorPower(motor, 0.3);
            else
                Subsystem.setMotorPower(motor, 0);

            telemetry.addData("a", gamepad1.a);
            telemetry.addData("dpad left", gamepad1.dpad_left);
            telemetry.addData("dpad right", gamepad1.dpad_right);
            telemetry.addData("dpad up", gamepad1.dpad_up);
            telemetry.addData("dpad down", gamepad1.dpad_down);
            telemetry.addData("arm servo current pwm", armServo.getPosition());
            telemetry.addData("grabber servo current pwm", grabberServo.getPosition());
            telemetry.addData("motor current encoder: ", motor.getCurrentPosition());
            telemetry.update();


        }
    }
}
