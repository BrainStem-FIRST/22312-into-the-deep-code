package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.Input;
import org.firstinspires.ftc.teamcode.util.PIDController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PIDLiftTele")
public class PIDLiftTele extends LinearOpMode {
    Input input;

    public void incrementPIDVar(PIDController pid, int editMode, double amount) {
        switch(editMode) {
            //case 0:
            //    pid.setkA(pid.getkA() + amount);
            //    break;
            case 1:
                pid.setkP(pid.getkP() + amount);
                break;
            case 2:
                pid.setkI(pid.getkI() + amount);
                break;
            case 3:
                pid.setkD(pid.getkD() + amount);
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        DcMotorEx liftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor");
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDController pid = new PIDController(0.01, 0, 0);
        pid.setInputBounds(0, 2000);
        pid.setOutputBounds(-1, 1);

        int editMode = 0; // 0 = kA, 1 = kP, etc

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

            double error = pid.getTarget() - liftMotor.getCurrentPosition();
            double kp = 0.005;
            double power = Range.clip(error * kp, -1, 1);


            if(input.getGamepadTracker1().isLeftBumperPressed() || input.getGamepadTracker1().isLeftTriggerPressed()) {
                Subsystem.setMotorPower(liftMotor, power);
            }

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
    }
}
