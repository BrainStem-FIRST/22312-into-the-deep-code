package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

import kotlin.OptIn;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class CollectingSystemExtensionTele extends LinearOpMode {

    private Input input;

    private BrainSTEMRobot robot;

    private boolean successfulSpitEnter = false;

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, AllianceColor.BLUE);


        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        //robot.setup();
        robot.setupCollectingSystem();

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

            listenForRobotControls();

            robot.update(dt);

            telemetry.addData("gamepad1 a down", input.getGamepadTracker1().isFirstFrameA());
            telemetry.addData("collecting system state, ", robot.getCollectingSystem().getStateManager().getActiveStateType());
            telemetry.addData("extension state, ",  robot.getExtension().getStateManager().getActiveStateType());
            telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            telemetry.addData("extension time", robot.getExtension().getStateManager().getState(Extension.StateType.RETRACTING).getTime());
            telemetry.addData("successfully entered spit", successfulSpitEnter);
            telemetry.update();
        }
    }

    private void listenForRobotControls() {
        listenForDriveTrainInput();
        listenForCollectionInput();
    }

    // without roadrunner
    private void listenForDriveTrainInput() {

        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y * -1;
        double rightStickX;
        double threshold = 0.1F;
        if (Math.abs(gamepad1.right_stick_x) > threshold) {
            if (gamepad1.right_stick_x < 0) {
                rightStickX = (gamepad1.right_stick_x * gamepad1.right_stick_x * -1 * (4.0 / 5.0) - (1.0 / 5.0));
            }
            else {
                rightStickX = (gamepad1.right_stick_x * gamepad1.right_stick_x * (4.0 / 5.0) + (1.0 / 5.0));
            }
        } else {
            rightStickX = 0;
        }
        if ((Math.abs(gamepad1.left_stick_y) > threshold) || (Math.abs(gamepad1.left_stick_x) > threshold) || Math.abs(gamepad1.right_stick_x) > threshold) {
            //Calculate formula for mecanum drive function
            double addValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) + leftStickX * Math.abs(leftStickX))))) / 100;
            double subtractValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) - leftStickX * Math.abs(leftStickX))))) / 100;


            //Set motor speed variables
            robot.getDriveTrain().setMotorPowers((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
        } else
            robot.getDriveTrain().stopMotors();
    }

    private void listenForCollectionInput() {

        // left and right bumpers move extension
        if (input.getGamepadTracker1().isRightBumperPressed()) {

            // enter search state if not already
            if (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN)
                robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH);

            // move extension out
            robot.getExtension().setTargetPower(Extension.SEARCH_POWER);
        }
        // move extension in
        else if (input.getGamepadTracker1().isLeftBumperPressed()) {
            robot.getExtension().setTargetPower(-Extension.SEARCH_POWER);
        }
        else {
            robot.getExtension().setTargetPower(0);
        }

        // right triggers toggle between (hinging down and collecting) and (hinging up and doing nothing)
        if (input.getGamepadTracker1().isFirstFrameRightTrigger()) {

            // hinge down and collect
            if (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.SEARCH)
                robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH_AND_COLLECT);

                // hinge up and stop collecting
            else if (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT)
                robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH);
        }

        // left trigger retracts
        if (input.getGamepadTracker1().isFirstFrameLeftTrigger())
            robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);

        // force spit in case block gets stuck
        if (input.getGamepadTracker2().isDpadDownPressed()) {
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.SPITTING_TEMP);
            ((SpitTempState) robot.getCollector().getStateManager().getState(Collector.StateType.SPITTING_TEMP)).continueRunning();
        }

        // force retraction for emergencies
        if (input.getGamepadTracker2().isDpadUpPressed())
            robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);
    }
}
