package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.Input;


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
        robot.setup();

        // TODO: delete code once done testing life
        robot.setBlockColorHeld(BlockColor.YELLOW);

        waitForStart();

        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;
        double interval = 1000/60.; // 60 fps
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

            // update custom input
            input.update();

            listenForRobotControls();

            robot.update(dt);

            telemetry.addData("robot x", robot.getDriveTrain().pose.position.x);
            telemetry.addData("robot y", robot.getDriveTrain().pose.position.y);
            telemetry.addData("robot vector real", robot.getDriveTrain().pose.heading.real);
            telemetry.addData("robot vector imaginary", robot.getDriveTrain().pose.heading.imag);
            telemetry.addData("robot angle", Math.atan2(robot.getDriveTrain().pose.heading.real, robot.getDriveTrain().pose.heading.imag));
            telemetry.addData("in pid mode", robot.getInPidMode());
            telemetry.addData("high basket", robot.isHighDeposit());
            telemetry.addData("high ram", robot.isHighRam());
            telemetry.addData("grabber has specimen", robot.getGrabber().getHasSpecimen());
            telemetry.addData("grabber has block", robot.getGrabber().getHasBlock());
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
                -gamepad1.right_stick_x
        ));
    }
    // without roadrunner
    private void listenForDriveTrainInputOld() {
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
    private void listenForLiftingInput() {
        // TODO: get rid of conditional below once lifting system and collecting system can work together
        //if(input.getGamepadTracker1().isFirstFrameX())
        //    robot.setInDepositingMode(!robot.getInDepositingMode());

        // checking changes in basket/bar heights
        if(input.getGamepadTracker2().isLeftBumperPressed()) {
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_BASKET);
            robot.setIsHighDeposit(true);
        }

        if(input.getGamepadTracker2().isLeftTriggerPressed()) {
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_BASKET);
            robot.setIsHighDeposit(false);
        }

        if(input.getGamepadTracker2().isRightBumperPressed())
            robot.setIsHighRam(true);

        // TODO: enable low ram setting input once figure out how to ram on low bar
        //if(input.getGamepadTracker2().isRightTriggerPressed())
            //robot.setIsHighRam(false);

        // button for "progressing" a state
        if(input.getGamepadTracker1().isFirstFrameA())
            switch(robot.getLiftingSystem().getStateManager().getActiveStateType()) {
                case TROUGH:
                    if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
                        if(robot.getBlockColorHeld() == BlockColor.YELLOW)
                        //if(robot.getInDepositingMode())
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_BASKET);
                        //else
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
                        // dropping block to human player station
                        if(!robot.getGrabber().getHasSpecimen()) {
                            robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
                            robot.getGrabber().setHasBlock(false);
                        }
                        // assuming robot is flush with submersible, so transitions to ram before
                        else
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_RAM);
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
