package org.firstinspires.ftc.teamcode.tele;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
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
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.CollectTempState;
import org.firstinspires.ftc.teamcode.robotStates.collectingSystem.collectorStates.SpitTempState;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.GamepadTracker;
import org.firstinspires.ftc.teamcode.util.Input;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class TeleMain extends LinearOpMode {
    public static class Params {
        public AllianceColor allianceColor = AllianceColor.BLUE;
        public double timeToHang = 100;
    }
    public static Params PARAMS = new Params();
    private double timeSinceStart;
    private Input input;
    private BrainSTEMRobot robot;
    private final Pose2d BEGIN_POSE = new Pose2d(-24, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        input = new Input(gamepad1, gamepad2);
        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, PARAMS.allianceColor, BEGIN_POSE);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        robot.setup();

        waitForStart();

        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;
        double maxDt = 0;
        double minDt = 10;
        timeSinceStart = 0; // time since start of match; need for hanging

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

            // for debugging: checking dt
            if(dt > maxDt)
                maxDt = dt;
            if(dt < minDt)
                minDt = dt;

            // for debugging
            telemetry.addData("", "");
            telemetry.addData("dt", dt);
            telemetry.addData("max dt", maxDt);
            telemetry.addData("min dt", minDt);

            // general stats
            telemetry.addData("time remaining", 120 - timeSinceStart);
            telemetry.addData("robot alliance", robot.getColorFromAlliance());

            // robot's pose
            telemetry.addData("", "");
            telemetry.addData("robot x", robot.getDriveTrain().pose.position.x);
            telemetry.addData("robot y", robot.getDriveTrain().pose.position.y);
            telemetry.addData("robot angle", robot.getDriveTrain().pose.heading.toDouble());

            // robot's lifting system
            telemetry.addData("", "");
            telemetry.addData("is depositing", robot.isDepositing());
            telemetry.addData("can transfer", robot.canTransfer());
            telemetry.addData("stay in trough", robot.getLiftingSystem().getStayInTrough());
            telemetry.addData("lifting system state", robot.getLiftingSystem().getStateManager().getActiveStateType());
            telemetry.addData("  lifting system's active state's time", robot.getLiftingSystem().getStateManager().getActiveState().getTime());
            telemetry.addData("lift state", robot.getLift().getStateManager().getActiveStateType());
            telemetry.addData("  lift's active state's time", robot.getLift().getStateManager().getActiveState().getTime());
            telemetry.addData("  lift goal encoder", robot.getLift().getTransitionState().getGoalStatePosition());
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
            telemetry.addData("extension state", robot.getExtension().getStateManager().getActiveStateType());
            telemetry.addData("  extension encoder", robot.getExtension().getExtensionMotor().getCurrentPosition());
            telemetry.addData("  extension target power", robot.getExtension().getTargetPower());
            telemetry.addData("  extension actual power", robot.getExtension().getExtensionMotor().getPower());
            telemetry.addData("  hitting extension hard stop", robot.getExtension().hitRetractHardStop());
            telemetry.addData("  magnet reset switch state", robot.getExtension().isMagnetSwitchActivated());

            // robot's hanging system
            telemetry.addData("", "");
            telemetry.addData("hanging state", robot.getHanger().getStateManager().getActiveStateType());
            telemetry.addData("  hang motor encoder", robot.getHanger().getHangMotor().getCurrentPosition());
            telemetry.addData("  hang goal encoder", robot.getHanger().getTransitionState().getGoalStatePosition());
            telemetry.update();
        }
    }

    private void listenForRobotControls() {
        listenForDriveTrainInput();
        listenForCollectionInput(input.getGamepadTracker1());
        listenForLiftingInput(); // checks both gamepad input in this function
        listenForHangingInput();
    }
    private void listenForDriveTrainInput() {
        // final double STRAFE_X_AMP = 0.3;
        final double STRAFE_Y_AMP = 0.5;
        final double TURN_AMP = 0.8;

        //int strafeDirX = input.getGamepadTracker1().isDpadUpPressed() ? 1 : input.getGamepadTracker1().isDpadDownPressed() ? -1 : 0;
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
            ));
    }
    private void listenForCollectionInput(@NonNull GamepadTracker gamepadTracker) {
        StateManager<CollectingSystem.StateType> collectingSystemManager = robot.getCollectingSystem().getStateManager();

        // go into search mode
        if (gamepadTracker.isRightBumperPressed() && collectingSystemManager.getActiveStateType() == CollectingSystem.StateType.IN)
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
    }
    private void listenForLiftingInput() {
        // checking changes in basket/bar heights
        if(input.getGamepadTracker2().isLeftBumperPressed()) {
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_BASKET);
            robot.setIsHighDeposit(true);
        }
        else if(input.getGamepadTracker2().isLeftTriggerPressed()) {
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_BASKET);
            robot.setIsHighDeposit(false);
        }
        // a = button for state progression
        if(input.getGamepadTracker1().isFirstFrameA() || input.getGamepadTracker2().isFirstFrameA())
            switch(robot.getLiftingSystem().getStateManager().getActiveStateType()) {
                case TROUGH:
                    // checking for bad transfer going down (resets lift back to trough pos)
                    if(robot.getLift().getTransitionState().getGoalStatePosition() == Lift.TROUGH_POS
                    && robot.getLift().getStateManager().getActiveStateType() != Lift.StateType.TROUGH_SAFETY)
                        robot.setCanTransfer(false);

                    // activating manual transfer
                    else if(!robot.canTransfer() && robot.getCollector().hasValidBlockColor())
                        robot.setCanTransfer(true); // I set to true so that next frame the execute in TroughState will lower lift and actually transfer block
                    // activating transition to deposit in basket once transfer is complete
                    else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY
                    && robot.isDepositing() && robot.getGrabber().hasBlock())
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_BASKET);
                    break;

                case TROUGH_TO_BASKET:
                    robot.getLiftingSystem().setButtonACued(true);
                    break;

                case BASKET_DEPOSIT:
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_TROUGH);
                    break;

                case DROP_AREA:
                    // what to do if grabber is closed
                    if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.CLOSED)
                        // if doesn't have specimen (means has block) then open grabber to drop off block
                        if(!robot.getGrabber().hasSpecimen()) {
                            robot.getGrabber().getTransitionState().setGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                            robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                        }
                        // if has specimen then proceed to prep for ram
                        else
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_RAM);
                    // grabbing specimen if grabber is open
                    else if(robot.getGrabber().getStateManager().getActiveStateType() == Grabber.StateType.OPEN) {
                        robot.getGrabber().getTransitionState().setGoalState(Grabber.CLOSE_POS, Grabber.StateType.CLOSED);
                        robot.getGrabber().setHasSpecimen(true);
                    }
                    break;

                case SPECIMEN_RAM:
                    // moving lift to ram specimen into bar
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.RAM_TO_TROUGH);
                    break;
            }

        // button for "going back" a state
        else if(input.getGamepadTracker1().isFirstFrameB())
            switch(robot.getLiftingSystem().getStateManager().getActiveStateType()) {
                case TROUGH:
                    if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
                        robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_DROP_AREA);
                case DROP_AREA:
                    // if grabber is closing or already closed, then open grabber (should run when fails to grab specimen)
                    if (robot.getGrabber().getTransitionState().getGoalStatePosition() == Grabber.CLOSE_POS) {
                        if(!robot.getGrabber().hasSpecimen())
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
                        else {
                            robot.getGrabber().getTransitionState().overrideGoalState(Grabber.OPEN_POS, Grabber.StateType.OPEN);
                            robot.getGrabber().setBlockColorHeld(BlockColor.NONE);
                        }
                    }
                    // resetting lifting system to trough for transfer again
                    else if (robot.getGrabber().getTransitionState().getGoalStatePosition() == Grabber.OPEN_POS) {
                        telemetry.addData("", "");
                        telemetry.addData("transitioning to drop area to trough", "");
                        telemetry.addData("", "");
                        robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
                    }
                    break;
            }
    }
    private void listenForHangingInput() {
        // automatically start raising hanging at 2:20
        //if (timeSinceStart >= PARAMS.timeToHang && robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.FULL_DOWN)
            //robot.getHanger().getTransitionState().setGoalState(Hanger.UP_TICK, Hanger.StateType.UP);

        // lower hanging to raise robot
        if (input.getGamepadTracker2().isRightBumperPressed()) {
            if (robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.FULL_DOWN)
                robot.getHanger().getTransitionState().setGoalState(Hanger.UP_TICK, Hanger.StateType.UP);
        }
        else if (input.getGamepadTracker2().isRightTriggerPressed()) {
            if (robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.UP)
                robot.getHanger().getTransitionState().setGoalState(Hanger.HANG_DOWN_TICK, Hanger.StateType.HANG_DOWN);
            else if (robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.TRANSITION
            && robot.getHanger().getTransitionState().getGoalStatePosition() == Hanger.UP_TICK)
                robot.getHanger().getTransitionState().overrideGoalState(Hanger.HANG_DOWN_TICK, Hanger.StateType.HANG_DOWN);
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
            robot.getDriveTrain().setMotorPowers((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
        } else {
            robot.getDriveTrain().stop();
        }
    }
}
