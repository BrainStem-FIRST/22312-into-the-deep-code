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
        public double kP = 0.01, kI = 0, kD = 0;
    }
    Input input;

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Params params = new Params();
        PIDController pid = new PIDController(params.kP, params.kI, params.kD);

        pid.setInputBounds(0, 2000);
        pid.setOutputBounds(-1, 1);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();


        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;
        double interval = 1000/60.; // 60 FPS
        double timeSinceLastUpdate = 0;


        while (opModeIsActive()) {
            // update dt
            prevTime = currentTime;
            currentTime = System.currentTimeMillis();
            dt = (currentTime - prevTime) / 1000.;
            timeSinceLastUpdate += dt;

            // update custom input
            input.update();

            if(input.getGamepadTracker1().isFirstFrameLeftBumper()) {
                pid.setTarget(2000);
                pid.reset();
            }
            if(input.getGamepadTracker1().isFirstFrameLeftTrigger()) {
                pid.setTarget(0);
                pid.reset();
            }

            doPrimitive(pid, liftMotor);
        }
    }
    public void doPrimitive(PIDController pid, DcMotorEx liftMotor) {
        double error = pid.getTarget() - liftMotor.getCurrentPosition();
        double kp = 0.005;
        double power = Range.clip(error * kp, -1, 1);

        Subsystem.setMotorPower(liftMotor, power);

        // checking for adjusting k values
        //if(input.getGamepadTracker2().isFirstFrameDpadUp())

        //else if(input.getGamepadTracker2().isFirstFrameDpadDown())

        telemetry.addData("lift encoder", liftMotor.getCurrentPosition());
        telemetry.addData("lift power", liftMotor.getPower());
        telemetry.addData("theoretical power", power);
        telemetry.addData("error", error);
        telemetry.addData("pid target", pid.getTarget());

        telemetry.update();
    }
    public void doActual(PIDController pid, DcMotorEx liftMotor) {
        double power = pid.update(liftMotor.getCurrentPosition());
        Subsystem.setMotorPower(liftMotor, power);

        telemetry.addData("lift encoder", liftMotor.getCurrentPosition());
        telemetry.addData("actual lift power", liftMotor.getPower());
        telemetry.addData("calculated power", power);
        telemetry.addData("error", pid.getTarget() - liftMotor.getCurrentPosition());
        telemetry.addData("pid target", pid.getTarget());

        telemetry.update();
    }
}
