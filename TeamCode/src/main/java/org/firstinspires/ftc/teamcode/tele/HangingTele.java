package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Hanger;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.Input;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "HangingTele")
public class HangingTele extends LinearOpMode {

    private Input input;

    private BrainSTEMRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        //robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, AllianceColor.BLUE);

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "HangMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        //robot.setupCollectingSystem(0);

        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;
        double interval = 1000/60.;
        double timeSinceLastUpdate = 0;

        while (opModeIsActive()) {

            /*
            telemetry.addData("lifting system state", robot.getLiftingSystem().getStateManager().getActiveStateType());
            telemetry.addData("lift state", robot.getLift().getStateManager().getActiveStateType());
            telemetry.addData("arm state", robot.getArm().getStateManager().getActiveStateType());
            telemetry.addData("grabber state", robot.getGrabber().getStateManager().getActiveStateType());
            */

            // update dt
            prevTime = currentTime;
            currentTime = System.currentTimeMillis();
            dt = (currentTime - prevTime) / 1000.;
            timeSinceLastUpdate += dt;

            //if (timeSinceLastUpdate < interval)
            //    continue;
            //timeSinceLastUpdate -= interval;

            // update custom input
            input.update();

            Subsystem.setMotorPower(motor, gamepad1.left_stick_y);
            //if (motor.getCurrentPosition() <= 0)
            //    Subsystem.setMotorPower(motor, 0);

            //robot.update(dt);

            //robot.update(dt);
            telemetry.addData("", "");
            //telemetry.addData("b pressed", input.getGamepadTracker1().isBPressed());
            telemetry.addData("motor encoder", motor.getCurrentPosition());
            //telemetry.addData("hanging state", robot.getHanger().getStateManager().getActiveStateType());
            //telemetry.addData("can enter going up hanging", robot.getHanger().getStateManager().tryEnterState(Hanger.StateType.GOING_UP));

            telemetry.update();
        }
    }
}
