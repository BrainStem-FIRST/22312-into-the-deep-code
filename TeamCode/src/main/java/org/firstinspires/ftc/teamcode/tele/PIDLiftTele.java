package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.Input;
import org.firstinspires.ftc.teamcode.util.PIDController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PIDLiftTele")
@Config
public class PIDLiftTele extends LinearOpMode {

    public static class Params {
        public double kP = 0.005, kI = 0, kD = 0;
    }
    public static Params PARAMS = new Params();
    Input input;

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDController pid = new PIDController(PARAMS.kP, PARAMS.kI, PARAMS.kD);

        pid.setInputBounds(0, 3400);
        pid.setOutputBounds(-1, 1);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // update custom input
            input.update();

            if(input.getGamepadTracker1().isFirstFrameLeftBumper()) {
                pid.setTarget(2500);
                pid.reset();
            }
            if(input.getGamepadTracker1().isFirstFrameLeftTrigger()) {
                pid.setTarget(500);
                pid.reset();
            }

            double power = pid.update(liftMotor.getCurrentPosition());

            telemetry.addData("power", power);
            telemetry.addData("motor encoder", liftMotor.getCurrentPosition());
            telemetry.addData("pid target", pid.getTarget());
            telemetry.addData("left bumper to set target to 2500", "");
            telemetry.addData("left trigger to set target to 500", "");
            telemetry.update();
            Subsystem.setMotorPower(liftMotor, power);
        }
    }
}
