package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "HangingTele")
public class HangingTele extends LinearOpMode {
    public static DcMotorSimple.Direction motorDirection = DcMotorSimple.Direction.REVERSE;
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "HangMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setDirection(motorDirection);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // checking input
            Subsystem.setMotorPower(motor, -gamepad1.left_stick_y);
            if(gamepad1.a)
                motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.addData("", "");
            telemetry.addData("motor direction", motor.getDirection());
            telemetry.addData("motor encoder", motor.getCurrentPosition());
            telemetry.addData("motor power", motor.getPower());
            telemetry.addData("motor current (milliAmps)", motor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.update();
        }
    }
}
