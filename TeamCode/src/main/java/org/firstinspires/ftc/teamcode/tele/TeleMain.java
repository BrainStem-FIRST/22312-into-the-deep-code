package org.firstinspires.ftc.teamcode.tele;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.GamepadTracker;
import org.firstinspires.ftc.teamcode.util.Input;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class TeleMain extends LinearOpMode {
    public static class Params {
        public AllianceColor allianceColor = AllianceColor.BLUE;
    }
    public static Params PARAMS = new Params();
    private Input input;
    private BrainSTEMRobot robot;
    private final Pose2d BEGIN_POSE = new Pose2d(-24, -7.5, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        input = new Input(gamepad1, gamepad2);
        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, PARAMS.allianceColor, BEGIN_POSE);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();

        waitForStart();

        robot.getStateManager().tryEnterState(BrainSTEMRobot.StateType.SETTING_UP); // setup robot

        long currentAbsoluteTime = System.currentTimeMillis();
        long prevAbsoluteTime;
        double dt;
        double maxDt = 0;
        double minDt = 10;
        double currentGameTime = 0;

        while (opModeIsActive()) {

            // update dt
            prevAbsoluteTime = currentAbsoluteTime;
            currentAbsoluteTime = System.currentTimeMillis();
            dt = (currentAbsoluteTime - prevAbsoluteTime) / 1000.;
            currentGameTime += dt;

            // update custom input
            input.update();

            // always allow driver control
            listenForDriveTrainInput();

            // only accept input if robot is playing
            if (robot.getStateManager().getActiveStateType() == BrainSTEMRobot.StateType.PLAYING) {
                listenForCollectionInput(input.getGamepadTracker1());
                listenForLiftingInput(); // checks both gamepad input in this function
            }

            // update robot
            robot.update(dt);

            // for debugging: checking dt
            if(dt > maxDt)
                maxDt = dt;
            if(dt < minDt)
                minDt = dt;

            // for debugging
            telemetry.addData("", "");
            telemetry.addData("game time", currentGameTime);
            telemetry.addData("dt", dt);
            telemetry.addData("max dt", maxDt);
            telemetry.addData("min dt", minDt);

            // general stats
            telemetry.addData("robot alliance", robot.getColorFromAlliance());

            // robot's pose
            telemetry.addData("", "");
            telemetry.addData("robot x", robot.getDriveTrain().pose.position.x);
            telemetry.addData("robot y", robot.getDriveTrain().pose.position.y);
            telemetry.addData("robot angle", robot.getDriveTrain().pose.heading.toDouble());
            telemetry.addData("robot state", robot.getStateManager().getActiveStateType());

            // robot's lifting system
            telemetry.addData("", "");
            telemetry.addData("lift need manual transfer", "");
            telemetry.addData("is high basket", robot.isHighDeposit());
            telemetry.addData("can enter basket to basket", robot.getLiftingSystem().getStateManager().getState(LiftingSystem.StateType.BASKET_TO_BASKET).canEnter());
            telemetry.addData("is basketDeposit done", robot.getLiftingSystem().getStateManager().getState(LiftingSystem.StateType.BASKET_DEPOSIT).isDone());
            telemetry.addData("is depositing", robot.isDepositing());
            telemetry.addData("can transfer", robot.canTransfer());
            telemetry.addData("stay in trough", robot.getLiftingSystem().getStayInTrough());
            telemetry.addData("lifting system state", robot.getLiftingSystem().getStateManager().getActiveStateType());
            telemetry.addData("  lifting system's active state's time", robot.getLiftingSystem().getStateManager().getActiveState().getTime());
            telemetry.addData("lift state", robot.getLift().getStateManager().getActiveStateType());
            telemetry.addData("  lift's active state's time", robot.getLift().getStateManager().getActiveState().getTime());
            telemetry.addData("  lift goal encoder", robot.getLift().getTransitionState().getGoalStatePosition());
            telemetry.addData("  lift goal state", robot.getLift().getTransitionState().getNextStateType());
            telemetry.addData("  lift motor encoder", robot.getLift().getLiftMotor().getCurrentPosition());
            telemetry.addData("  lift motor power", robot.getLift().getLiftMotor().getPower());
            telemetry.addData("  lift transition pid", robot.getLift().getTransitionState().getPid().toString());
            telemetry.addData("arm state", robot.getArm().getStateManager().getActiveStateType());
            telemetry.addData("  arm's active state's time", robot.getArm().getStateManager().getActiveState().getTime());
            telemetry.addData("grabber state", robot.getGrabber().getStateManager().getActiveStateType());
            telemetry.addData("  grabber's active state's time", robot.getGrabber().getStateManager().getActiveState().getTime());
            telemetry.addData("  grabber has specimen", robot.getGrabber().hasSpecimen());
            telemetry.addData("  grabber block color", robot.getGrabber().getBlockColorHeld());

            // robot's collecting system
            telemetry.addData("", "");
            telemetry.addData("can collect", robot.canCollect());
            telemetry.addData("collecting system state", robot.getCollectingSystem().getStateManager().getActiveStateType());
            telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            telemetry.addData("  collector motor current", robot.getCollector().getSpindleMotor().getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("  validated block color sensor", robot.getCollector().getBlockColorSensor().getValidatedColor());
            telemetry.addData("  block color in trough", robot.getCollector().getBlockColorInTrough());
            telemetry.addData("hinge state", robot.getHinge().getStateManager().getActiveStateType());
            telemetry.addData("  hinge goal pwm", robot.getHinge().getTransitionState().getGoalStatePosition());
            telemetry.addData("extension state", robot.getExtension().getStateManager().getActiveStateType());
            telemetry.addData("  extension encoder", robot.getExtension().getExtensionMotor().getCurrentPosition());
            telemetry.addData("  extension target power", robot.getExtension().getTargetPower());
            telemetry.addData("  extension actual power", robot.getExtension().getExtensionMotor().getPower());
            telemetry.addData("  hitting extension hard stop", robot.getExtension().hitRetractHardStop());
            telemetry.addData("  magnet reset switch state", robot.getExtension().isMagnetSwitchActivated());
            telemetry.addData(" raw magnet sensor state", robot.getExtension().getMagnetSwitch().getState());

            // robot's hanging system
            telemetry.addData("", "");
            telemetry.addData("hanging state", robot.getHanger().getStateManager().getActiveStateType());
            telemetry.addData("  hang motor power", robot.getHanger().getHangMotor().getPower());
            telemetry.addData("  hang motor encoder", robot.getHanger().getHangMotor().getCurrentPosition());
            telemetry.addData("  hang goal encoder", robot.getHanger().getTransitionState().getGoalStatePosition());
            telemetry.addData("  hang goal state", robot.getHanger().getTransitionState().getNextStateType());

            telemetry.update();
        }
    }
    private void listenForDriveTrainInput() {
        final double STRAFE_Y_AMP = 0.8;
        final double TURN_AMP = 0.8;
        final double hangAndExtendPower = 0.2;

        if (robot.getLift().getTransitionState().getNextStateType() == Lift.StateType.RAM_AFTER
        && robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.SEARCH)
            robot.getDriveTrain().setDrivePowers(new PoseVelocity2d(new Vector2d(hangAndExtendPower, 0), 0));

        int strafeDirY = input.getGamepadTracker1().isDpadRightPressed() ? 1 : input.getGamepadTracker1().isDpadLeftPressed() ? -1 : 0;

        if (strafeDirY != 0)
            robot.getDriveTrain().setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0, -strafeDirY * STRAFE_Y_AMP),
                    0
            ));
        else
            robot.getDriveTrain().setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x * TURN_AMP
            ));;
    }
    private void listenForCollectionInput(@NonNull GamepadTracker gamepadTracker) {
        StateManager<CollectingSystem.StateType> collectingSystemManager = robot.getCollectingSystem().getStateManager();

        // go into search mode
        if (gamepadTracker.isRightBumperPressed()
                && (collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.IN
                || collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.RETRACTING))
            collectingSystemManager.tryEnterState(CollectingSystem.StateType.SEARCH);

        // set extension target power
        if (collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.SEARCH ||
                collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT)
            if (input.getGamepadTracker1().isRightBumperPressed())
                robot.getExtension().setTargetPower(Extension.SEARCH_POWER);
            else if ((input.getGamepadTracker1().isLeftBumperPressed())
                    && (robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.SEARCH
                    || robot.getExtension().getExtensionMotor().getCurrentPosition() > Extension.MIN_SEARCH_AND_COLLECT_POSITION))
                robot.getExtension().setTargetPower(-Extension.SEARCH_POWER);
            else
                robot.getExtension().setTargetPower(0);

        // right trigger toggle between (hinging down and collecting) and (hinging up and doing nothing)
        // or do a short extension and collection
        if (gamepadTracker.isFirstFrameRightTrigger())

            // short extension and collection
            if (collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.IN)
                collectingSystemManager.tryEnterState(CollectingSystem.StateType.SHORT_EXTEND);

            // go to search and collect mode
            else if (collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.SEARCH)
                collectingSystemManager.tryEnterState(CollectingSystem.StateType.SEARCH_AND_COLLECT);

                // go to search mode
            else if (collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.SEARCH_AND_COLLECT)
                collectingSystemManager.tryEnterState(CollectingSystem.StateType.SEARCH);

        // left trigger retracts
        if (gamepadTracker.isFirstFrameLeftTrigger())
            collectingSystemManager.tryEnterState(CollectingSystem.StateType.RETRACTING);

        // force spit in case block gets stuck - spits as long as gamepad up is pressed
        if (gamepadTracker.isDpadUpPressed())
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.SPITTING_TEMP);

        // force collect in case block is imperfectly collected - collects as long as gamepad down is pressed
        if (gamepadTracker.isDpadDownPressed())
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.COLLECTING_TEMP);

        // short extend while hanging
        if (gamepadTracker.isAPressed()
                && robot.getLift().getTransitionState().getNextStateType() == Lift.StateType.RAM_AFTER)
            robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.SEARCH);

    }
    private void listenForLiftingInput() {
        // checking changes in basket/bar heights
        if(input.getGamepadTracker2().isLeftBumperPressed()) {
            robot.setIsHighDeposit(true);
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_BASKET);
        }
        else if(input.getGamepadTracker2().isLeftTriggerPressed()) {
            robot.setIsHighDeposit(false);
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_BASKET);
        }
        // a = button for state progression
        // b = button for state regression
        // x = button for safety overrides
        switch(robot.getLiftingSystem().getStateManager().getActiveStateType()) {
            case TROUGH:
                // checking for transition to basket
                if (input.getGamepadTracker2().isFirstFrameA()
                && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY
                && robot.isDepositing() && robot.getGrabber().hasBlock())
                        robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_BASKET);

                // activating manual transfer if everything properly setup
                else if ((input.getGamepadTracker2().isFirstFrameA() || input.getGamepadTracker1().isFirstFrameA())
                && !robot.canTransfer() && robot.getArm().getStateManager().getActiveStateType() == Arm.StateType.TRANSFER && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
                        robot.setCanTransfer(true); // I set to true so that next frame the execute in TroughState will lower lift and actually transfer block

                // transitioning to drop area if need to
                if(input.getGamepadTracker1().isFirstFrameB()
                && robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_DROP_AREA);

                // handling block knocking check
                if ((input.getGamepadTracker1().isFirstFrameX() || input.getGamepadTracker2().isFirstFrameX())
                && robot.getCollectingSystem().getStateManager().getActiveStateType() == CollectingSystem.StateType.IN) {
                    robot.setCanTransfer(false);
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.KNOCK_BLOCK);
                }
                break;

            case TROUGH_TO_BASKET:
                if(input.getGamepadTracker2().isFirstFrameA())
                    robot.getLiftingSystem().setButtonACued(true);
                break;

            case BASKET_DEPOSIT:
                if(input.getGamepadTracker2().isFirstFrameA())
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_DROP_AREA);
                break;

            case TROUGH_TO_DROP_AREA:
                if (input.getGamepadTracker2().isFirstFrameB())
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
                break;

            case DROP_AREA:
                if (input.getGamepadTracker1().isFirstFrameA()) {
                    // what to do if grabber is closed
                    if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED)
                        // if doesn't have specimen (means has block) then open grabber to drop off block
                        if (!robot.getGrabber().hasSpecimen()) {
                            robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                            robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                        }
                        // if has specimen then proceed to prep for ram
                        else
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_RAM);
                        // grabbing specimen if grabber is open
                    else if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                        robot.getGrabber().getTransitionState().setGoalState(Grabber.CLOSE_POS, Grabber.StateType.CLOSED);
                        robot.getGrabber().setHasSpecimen(true);
                    }
                }

                // if grabber is closing or already closed, then open grabber (should run when fails to grab specimen)
                if (robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED) {
                    // transitioning to trough for deposit bc u have block
                    if(!robot.getGrabber().hasSpecimen() && input.getGamepadTracker2().isFirstFrameB())
                        robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
                    else if (robot.getGrabber().hasSpecimen() && input.getGamepadTracker1().isFirstFrameB()){
                        robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                        robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                    }
                }
                // resetting lifting system to trough for transfer again
                else if ((input.getGamepadTracker2().isFirstFrameB() || input.getGamepadTracker1().isFirstFrameB())
                && robot.getGrabber().getTransitionState().getGoalStatePosition() == Grabber.OPEN_POS) {
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
                }

                break;

            case DROP_AREA_TO_TROUGH:
                // checking for overriding to immediately transition to basket to save time
                if(input.getGamepadTracker2().isFirstFrameA()
                && robot.getGrabber().hasBlock())
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_BASKET);
                break;

            case SPECIMEN_RAM:
                // moving lift to ram specimen into bar
                if(input.getGamepadTracker1().isFirstFrameA())
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.RAM_TO_TROUGH);
                break;
        }
    }

    private void listenForDriveTrainInputOld() {
        // drivetrain
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
            //robot.getDriveTrain().setMotorPowers((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
        } else {
            //robot.getDriveTrain().stop();
        }
    }
}
