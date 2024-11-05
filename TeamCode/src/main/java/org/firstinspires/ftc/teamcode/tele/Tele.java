package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class Tele extends LinearOpMode {

    private Input input;

    private BrainSTEMRobot robot;


    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, input, AllianceColor.BLUE);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        robot.setup();

        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;
        double interval = 1000/60.;
        double timeSinceLastUpdate = 0;

        while (opModeIsActive()) {
            telemetry.addData("a", gamepad1.a);
            telemetry.addData("b", gamepad1.b);
            telemetry.addData("y", gamepad1.y);
            telemetry.addData("collecting system state, ", robot.getCollectingSystem().getStateManager().getActiveStateType());
            telemetry.addData("extension state, ",  robot.getExtension().getStateManager().getActiveStateType());
            telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            telemetry.addData("lifting system state", robot.getLiftingSystem().getStateManager().getActiveStateType());
            telemetry.addData("lift state", robot.getLift().getStateManager().getActiveStateType());
            telemetry.addData("arm state", robot.getArm().getStateManager().getActiveStateType());
            telemetry.addData("grabber state", robot.getGrabber().getStateManager().getActiveStateType());

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

            listenForCollectionInput();
            robot.update(dt);
            telemetry.update();
        }
    }

    private void listenForRobotControls() {
        telemetry.addData("inside listen for robot controls", "");
        //listenForDriveTrainInput();
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
            } else {
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
            robot.getDriveTrain().setDrivePower((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
        } else
            robot.getDriveTrain().stop();
    }


    private void listenForCollectionInput() {
        if (input.getGamepadTracker1().isFirstFrameA()) {
            // goes from in to search, then toggles between search and search and collect
            switch (robot.getCollectingSystem().getStateManager().getActiveStateType()) {
                case IN:
                case SEARCH_AND_COLLECT:
                    robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH);
                    break;
                case SEARCH:
                    robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH_AND_COLLECT);
                    break;
            }
        }
        if (input.getGamepadTracker1().isFirstFrameB()) {
            robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);
        }
    }
}
