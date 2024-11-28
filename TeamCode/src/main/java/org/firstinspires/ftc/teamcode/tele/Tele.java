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
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.CollectTempState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.Input;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class Tele extends LinearOpMode {

    private final double TURN_AMP = 0.4;

    private Input input;
    private BrainSTEMRobot robot;

    private double timeSinceStart;

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, AllianceColor.RED);


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
            robot.getDriveTrain().updatePoseEstimate();

            telemetry.addData("magnet reset switch state", robot.getExtension().isMagnetSwitchActivated());

            telemetry.addData("robot alliance", robot.getColorFromAlliance());
            telemetry.addData("robot color held", robot.getBlockColorHeld());
            telemetry.addData("", "");
            telemetry.addData("b pressed", input.getGamepadTracker1().isBPressed());
            telemetry.addData("hanging state", robot.getHanger().getStateManager().getActiveStateType());

            telemetry.addData("a first down", input.getGamepadTracker1().isFirstFrameA());
            telemetry.addData("a down", input.getGamepadTracker1().isAPressed());
            telemetry.addData("", "");
            telemetry.addData("robot x", robot.getDriveTrain().pose.position.x);
            telemetry.addData("robot y", robot.getDriveTrain().pose.position.y);
            telemetry.addData("robot vector real", robot.getDriveTrain().pose.heading.real);
            telemetry.addData("robot vector imaginary", robot.getDriveTrain().pose.heading.imag);
            telemetry.addData("robot angle", robot.getDriveTrain().pose.heading.toDouble());
            telemetry.addData("", "");
            telemetry.addData("in pid mode", robot.getInPidMode());
            telemetry.addData("", "");
            telemetry.addData("grabber has specimen", robot.getGrabber().getHasSpecimen());
            telemetry.addData("grabber has block", robot.getGrabber().getHasBlock());
            telemetry.addData("lifting system state", robot.getLiftingSystem().getStateManager().getActiveStateType());
            telemetry.addData("lift state", robot.getLift().getStateManager().getActiveStateType());
            telemetry.addData("lift goal position", robot.getLift().getTransitionState().getGoalStatePosition());
            telemetry.addData("lift motor encoder", robot.getLift().getLiftMotor().getCurrentPosition());
            telemetry.addData("lift error", robot.getLift().getPid().getTarget() - robot.getLift().getLiftMotor().getCurrentPosition());
            telemetry.addData("lift motor power", robot.getLift().getLiftMotor().getPower());
            telemetry.addData("arm state", robot.getArm().getStateManager().getActiveStateType());
            telemetry.addData("grabber state", robot.getGrabber().getStateManager().getActiveStateType());
            telemetry.addData("", "");
            telemetry.addData("collecting system state", robot.getCollectingSystem().getStateManager().getActiveStateType());
            telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            telemetry.addData("hinge state", robot.getHinge().getStateManager().getActiveStateType());
            telemetry.addData("extension state", robot.getExtension().getStateManager().getActiveStateType());
            telemetry.addData("extension encoder", robot.getExtension().getExtensionMotor().getCurrentPosition());
            telemetry.addData("hitting extension hard stop", robot.getExtension().hitRetractHardStop());
            telemetry.addData("has red", robot.getCollector().getBlockColorSensor().hasColor(BlockColor.RED));
            telemetry.addData("has blue", robot.getCollector().getBlockColorSensor().hasColor(BlockColor.BLUE));
            telemetry.addData("has yellow", robot.getCollector().getBlockColorSensor().hasColor(BlockColor.YELLOW));
            telemetry.update();
        }
    }

    private void listenForRobotControls() {
        listenForDriveTrainInput();
        listenForCollectionInput();
        listenForLiftingInput();
        listenForHangingInput();

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
            else if (input.getGamepadTracker1().isLeftBumperPressed()
                    && (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.SEARCH
                    || robot.getExtension().getExtensionMotor().getCurrentPosition() > Extension.MIN_SEARCH_AND_COLLECT_POSITION))
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

        // force spit in case block gets stuck - spits as long as gamepad up is pressed
        if (input.getGamepadTracker1().isDpadUpPressed()) {
            collectorManager.tryEnterState(Collector.StateType.SPITTING_TEMP);
            ((SpitTempState) collectorManager.getState(Collector.StateType.SPITTING_TEMP)).continueRunning();
        }

        // force collect in case block is imperfectly collected - collects as long as gamepad down is pressed
        if (input.getGamepadTracker2().isDpadDownPressed()) {
            collectorManager.tryEnterState(Collector.StateType.COLLECTING_TEMP);
            ((CollectTempState) collectorManager.getState(Collector.StateType.COLLECTING_TEMP)).continueRunning();
        }
    }
    private void listenForLiftingInput() {
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
                        // dropping block to human player station
                        if(!robot.getGrabber().getHasSpecimen()) {
                            robot.getGrabber().getStateManager().tryEnterState(Grabber.StateType.OPENING);
                            robot.getGrabber().setHasBlock(false);
                        }
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
            // backtracking if fail to grab specimen
            if(robot.getLiftingSystem().getStateManager().getActiveStateType() == LiftingSystem.StateType.SPECIMEN_RAM &&
                    robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_BEFORE) {
                robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.RAM_TO_DROP_AREA);
                robot.getGrabber().setHasSpecimen(false);
            }
    }
    // TODO: if press right trigger and not extended, then extend constant length and colect
    // TODO: only fully hinge parallel once collector is past submersible to prevent sag from causing back of collector to hit submersible wall
    private void listenForHangingInput() {
        //temp code
        //if (input.getGamepadTracker1().isBPressed() && robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.FULL_DOWN)
        //    robot.getHanger().getStateManager().tryEnterState(Hanger.StateType.GOING_UP);

        // automatically start raising hanging at 2:20
        //if (timeSinceStart >= 140 && robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.FULL_DOWN)
        //    robot.getHanger().getStateManager().tryEnterState(Hanger.StateType.GOING_UP);
        // lower hanging to raise robot
        if (input.getGamepadTracker2().isFirstFrameA())
            if(robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.FULL_DOWN) {
                robot.getHanger().getTransitionState().setGoalState(Hanger.UP_TICK, Hanger.StateType.UP);
            }
            else if(robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.UP) {
                robot.getHanger().getTransitionState().setGoalState(Hanger.HANG_DOWN_TICK, Hanger.StateType.HANG_DOWN);
            }
    }
}
