package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.Input;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CollectingSystemTele")
public class CollectingSystemExtensionTele extends LinearOpMode {

    private Input input;

    private BrainSTEMRobot robot;

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
            telemetry.addData("hinge state", robot.getHinge().getStateManager().getActiveStateType());
            telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            telemetry.addData("extension target power", robot.getExtension().getTargetPower());
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
        StateManager<CollectingSystem.StateType> collectingSystemManager = robot.getCollectingSystem().getStateManager();
        StateManager<Collector.StateType> collectorManager = robot.getCollector().getStateManager();

        // go into search mode
        if (input.getGamepadTracker1().isRightBumperPressed() && collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.IN)
            collectingSystemManager.tryEnterState(CollectingSystem.StateType.SEARCH);

        // set extension target power
        if (collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.SEARCH ||
                collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT)
            if (input.getGamepadTracker1().isRightBumperPressed())
                robot.getExtension().setTargetPower(Extension.SEARCH_POWER);
            else if (input.getGamepadTracker1().isLeftBumperPressed())
                robot.getExtension().setTargetPower(-Extension.SEARCH_POWER);
            else
                robot.getExtension().setTargetPower(0);
        else
            robot.getExtension().setTargetPower(0);

        // right trigger toggle between (hinging down and collecting) and (hinging up and doing nothing)
        if (input.getGamepadTracker1().isFirstFrameRightTrigger()) {

            // go to search and collect mode
            if (collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.SEARCH)
                collectingSystemManager.tryEnterState(CollectingSystem.StateType.SEARCH_AND_COLLECT);

            // go to search mode
            else if (collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT)
                collectingSystemManager.tryEnterState(CollectingSystem.StateType.SEARCH);
        }

        // left trigger retracts
        if (input.getGamepadTracker1().isFirstFrameLeftTrigger())
            collectingSystemManager.tryEnterState(CollectingSystem.StateType.RETRACTING);

        // force spit in case block gets stuck - spits as long as gamepad button is down
        if (input.getGamepadTracker1().isDpadUpPressed()) {
            collectorManager.tryEnterState(Collector.StateType.SPITTING_TEMP);
            ((SpitTempState) collectorManager.getState(Collector.StateType.SPITTING_TEMP)).continueRunning();
        }
    }
}
