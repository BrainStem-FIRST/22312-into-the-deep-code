package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Hanger;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.Input;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CollectingExtensionTele")
public class CollectingSystemExtensionTele extends LinearOpMode {
    private final double TURN_AMP = 0.8;

    private Input input;
    private BrainSTEMRobot robot;

    private double timeSinceStart;

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED);


        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        // setup assumes all motors are good
        robot.setup();

        waitForStart();

        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;
        double interval = 1000/60.; // 60 fps
        timeSinceStart = 0; // time since start of match

        while (opModeIsActive()) {
            // update dt
            prevTime = currentTime;
            currentTime = System.currentTimeMillis();
            dt = (currentTime - prevTime) / 1000.;
            timeSinceStart += dt;

            // update custom input
            input.update();

            listenForRobotControls();

            robot.update(dt);

            telemetry.addData("robot alliance", robot.getColorFromAlliance());
            telemetry.addData("robot color held", robot.getBlockColorHeld());
            telemetry.addData("color sensor color", robot.getCollector().getBlockColorSensor().getBlockColor());
            telemetry.addData("", "");
            telemetry.addData("b pressed", input.getGamepadTracker1().isBPressed());
            telemetry.addData("in pid mode", robot.getInPidMode());
            telemetry.addData("", "");
            telemetry.addData("collecting system state", robot.getCollectingSystem().getStateManager().getActiveStateType());
            telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            telemetry.addData("hinge state", robot.getHinge().getStateManager().getActiveStateType());
            telemetry.addData("extension state", robot.getExtension().getStateManager().getActiveStateType());
            telemetry.addData("extension encoder", robot.getExtension().getExtensionMotor().getCurrentPosition());
            telemetry.addData("extension motor power", robot.getExtension().getExtensionMotor().getPower());

            telemetry.update();
        }
    }

    private void listenForRobotControls() {
        listenForDriveTrainInput();
        listenForCollectionInput();

        // x toggles pid actions
        if(input.getGamepadTracker1().isFirstFrameX())
            robot.setInPidMode(!robot.getInPidMode());
    }

    private void listenForDriveTrainInput() {
        robot.getDriveTrain().setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x
                ),
                -gamepad1.right_stick_x * TURN_AMP
        ));
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
