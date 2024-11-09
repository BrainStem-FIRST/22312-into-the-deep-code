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
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

import kotlin.OptIn;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class Tele extends LinearOpMode {

    private Input input;

    private BrainSTEMRobot robot;

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, AllianceColor.BLUE);


        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();

        while(!robot.setup()) {
            telemetry.addData("robot status", "setting up");
            telemetry.update();
        }

        // TODO: delete code once done testing life
        robot.setBlockColorHeld(BlockColor.YELLOW);

        waitForStart();

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

            telemetry.addData("gamepad1 a down", input.getGamepadTracker1().isAPressed());
            //telemetry.addData("collecting system state, ", robot.getCollectingSystem().getStateManager().getActiveStateType());
            //telemetry.addData("extension state, ",  robot.getExtension().getStateManager().getActiveStateType());
            //telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            //telemetry.addData("extension time", robot.getExtension().getStateManager().getState(Extension.StateType.RETRACTING).getTime());
            telemetry.addData("high basket", robot.isHighDeposit());
            telemetry.addData("lifting system state", robot.getLiftingSystem().getStateManager().getActiveStateType());
            telemetry.addData("lift state", robot.getLift().getStateManager().getActiveStateType());
            telemetry.addData("arm state", robot.getArm().getStateManager().getActiveStateType());
            telemetry.addData("grabber state", robot.getGrabber().getStateManager().getActiveStateType());
            telemetry.addData("lift motor encoder", robot.getLift().getLiftMotor().getCurrentPosition());
            telemetry.update();
        }
    }

    private void listenForRobotControls() {
        listenForDriveTrainInput();
        listenForCollectionInput();
        listenForLiftingInput();
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
            robot.getCollector().useSpittingSafety(false);
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.SPITTING);
        }

        // force retraction for emergencies
        if (input.getGamepadTracker2().isDpadUpPressed())
            robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);
    }

    private void listenForLiftingInput() {
        // checking changes in basket/bar heights
        // TODO: make so if arm is already dropped then raise back up and drop back drop (USE LiftingSystem.BASKET_TO_BASKET state)
        if(input.getGamepadTracker2().isLeftBumperPressed()) {
            if(robot.getLift().getTransitionState().getGoalStatePosition() == Lift.LOW_BASKET_POS) // will be true during transition to that position as well as when is already at position
                robot.getLift().getTransitionState().overrideGoalState(Lift.HIGH_BASKET_POS);
            robot.setIsHighDeposit(true);
        }

        if(input.getGamepadTracker2().isLeftTriggerPressed()) {
            if(robot.getLift().getTransitionState().getGoalStatePosition() == Lift.HIGH_BASKET_POS)
                robot.getLift().getTransitionState().overrideGoalState(Lift.LOW_BASKET_POS);
            robot.setIsHighDeposit(false);

        }

        if(input.getGamepadTracker2().isRightBumperPressed())
            robot.setIsHighRam(true);
        if(input.getGamepadTracker2().isLeftBumperPressed())
            robot.setIsHighRam(false);

        // button for "progressing" a state
        if(input.getGamepadTracker1().isFirstFrameA())
            switch(robot.getLiftingSystem().getStateManager().getActiveStateType()) {
                case TROUGH:
                    if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
                        if(robot.getBlockColorHeld() == BlockColor.YELLOW)
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_BASKET);
                        else if(robot.getBlockColorHeld() == robot.getColorFromAlliance())
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_DROP_AREA);
                    break;
                case BASKET_DEPOSIT:
                    robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
                    robot.getGrabber().setHasBlock(false);
                    robot.setBlockColorHeld(BlockColor.NONE);
                    break;
                case DROP_AREA:
                    if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                        robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
                        robot.getGrabber().setHasBlock(false);
                        // technically should set robot.blockColorHeld to false, but it will immediately be set to true when specimen is picked up, so no point of changing value
                    }
                    else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                        robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.CLOSING);
                        robot.getGrabber().setHasSpecimen(true);
                    }
                    break;
                case SPECIMEN_RAM:
                    if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_BEFORE)
                        robot.getLift().getTransitionState().setGoalState(robot.getLift().getRamAfterPos(), Lift.StateType.RAM_AFTER);
                    break;
            }

        // button for "going back" a state
        else if(input.getGamepadTracker1().isFirstFrameB())
            if(robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.DROP_AREA)
                // toggling grabber
                if(!robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.CLOSING))
                    robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
    }


    private void listenForCollectionInputOld() {

        // update states
        if (input.getGamepadTracker1().isFirstFrameA()) {
            // goes from in to search, then toggles between search and search and collect
            switch (robot.getCollectingSystem().getStateManager().getActiveStateType()) {
                case IN:
                    robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH);
                    break;
                case SEARCH:
                    robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH_AND_COLLECT);
                    break;
                case SEARCH_AND_COLLECT:
                    robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);
                    break;
            }
        }
        if (input.getGamepadTracker1().isFirstFrameB()) {
            // if the collector is collecting, spit
            if(robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT)
                robot.getCollector().getStateManager().tryEnterState(Collector.StateType.SPITTING);
            // if the collector is doing nothing, retract
            else if (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.SEARCH)
                robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);
        }

        // update extension target power (how fast it moves)
        robot.getExtension().setTargetPower(input.getGamepadTracker1().getGamepad().right_stick_y * -0.6);
    }
}
