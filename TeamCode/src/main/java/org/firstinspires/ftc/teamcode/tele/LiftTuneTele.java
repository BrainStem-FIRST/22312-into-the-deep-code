package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LiftTuneTele")
public class LiftTuneTele extends LinearOpMode {
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
                armServo.setPosition(Arm.TRANSFER_POS);
            else if(gamepad1.b)
                armServo.setPosition(Arm.SPECIMEN_HANG_POS);
            else if(gamepad1.y)
                armServo.setPosition(Arm.BASKET_SAFETY_POS);
            else if(gamepad1.x)
                armServo.setPosition(Arm.BLOCK_DROP_POS);

            if(gamepad1.dpad_left)
                grabberServo.setPosition(0.99);
            else if(gamepad1.dpad_right)
                grabberServo.setPosition(0.01);

            if((gamepad1.left_stick_y > 0.2 && motor.getCurrentPosition() > Lift.ABSOLUTE_MIN + 10) || // checking down movement
                (gamepad1.left_stick_y < -0.2 && motor.getCurrentPosition() < Lift.ABSOLUTE_MAX - 10)) // checking up movement
                Subsystem.setMotorPower(motor, -gamepad1.left_stick_y);
            else
                Subsystem.setMotorPower(motor, 0.05);

            telemetry.addData("a", gamepad1.a);
            telemetry.addData("left stick y", gamepad1.left_stick_y);
            telemetry.addData("dpad up", gamepad1.dpad_up);
            telemetry.addData("dpad down", gamepad1.dpad_down);
            telemetry.addData("arm servo current pwm", armServo.getPosition());
            telemetry.addData("grabber servo current pwm", grabberServo.getPosition());
            telemetry.addData("motor current encoder: ", motor.getCurrentPosition());
            telemetry.update();


        }
    }
}
