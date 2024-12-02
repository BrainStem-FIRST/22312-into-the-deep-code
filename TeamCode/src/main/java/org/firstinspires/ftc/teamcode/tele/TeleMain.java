package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class TeleMain extends LinearOpMode {
    public static class Params {
        public AllianceColor allianceColor = AllianceColor.RED;
    }
    public Params PARAMS = new Params();
    private Input input;
    private BrainSTEMRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        input = new Input(gamepad1, gamepad2);
        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, PARAMS.allianceColor);


        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        robot.setup();

        waitForStart();

        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;
        double timeSinceStart = 0; // time since start of match; need for hanging

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

            telemetry.addData("time remaining", 120 - timeSinceStart);
            telemetry.addData("robot alliance", robot.getColorFromAlliance());

            telemetry.addData("", "");
            telemetry.addData("robot x", robot.getDriveTrain().pose.position.x);
            telemetry.addData("robot y", robot.getDriveTrain().pose.position.y);
            telemetry.addData("robot angle", robot.getDriveTrain().pose.heading.toDouble());

            telemetry.addData("", "");
            telemetry.addData("is depositing", robot.isDepositing());
            telemetry.addData("can transfer", robot.canTransfer());
            telemetry.addData("lifting system state", robot.getLiftingSystem().getStateManager().getActiveStateType());
            telemetry.addData("lift state", robot.getLift().getStateManager().getActiveStateType());
            telemetry.addData("lift goal position", robot.getLift().getTransitionState().getGoalStatePosition());
            telemetry.addData("lift motor encoder", robot.getLift().getLiftMotor().getCurrentPosition());
            telemetry.addData("lift error", robot.getLift().getPid().getTarget() - robot.getLift().getLiftMotor().getCurrentPosition());
            telemetry.addData("lift motor power", robot.getLift().getLiftMotor().getPower());
            telemetry.addData("arm state", robot.getArm().getStateManager().getActiveStateType());
            telemetry.addData("grabber state", robot.getGrabber().getStateManager().getActiveStateType());
            telemetry.addData("grabber has specimen", robot.getGrabber().hasSpecimen());
            telemetry.addData("grabber block color", robot.getGrabber().getBlockColorHeld());

            telemetry.addData("", "");
            telemetry.addData("block color in trough", robot.getCollector().getBlockColorInTrough());
            telemetry.addData("can collect", robot.canCollect());
            telemetry.addData("collecting system state", robot.getCollectingSystem().getStateManager().getActiveStateType());
            telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            telemetry.addData("hinge state", robot.getHinge().getStateManager().getActiveStateType());
            telemetry.addData("extension state", robot.getExtension().getStateManager().getActiveStateType());
            telemetry.addData("extension encoder", robot.getExtension().getExtensionMotor().getCurrentPosition());
            telemetry.addData("extension target power", robot.getExtension().getTargetPower());
            telemetry.addData("extension actual power", robot.getExtension().getExtensionMotor().getPower());
            telemetry.addData("hitting extension hard stop", robot.getExtension().hitRetractHardStop());
            telemetry.addData("validated block color sensor", robot.getCollector().getBlockColorSensor().getValidatedColor());
            telemetry.addData("magnet reset switch state", robot.getExtension().isMagnetSwitchActivated());

            telemetry.addData("", "");
            telemetry.addData("hanging state", robot.getHanger().getStateManager().getActiveStateType());
            telemetry.addData("hanging motor encoder", robot.getHanger().getHangMotor().getCurrentPosition());
            telemetry.update();
        }
    }

    private void listenForRobotControls() {
        listenForDriveTrainInput();
        listenForCollectionInput();
        listenForLiftingInput();
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

        // right trigger toggle between (hinging down and collecting) and (hinging up and doing nothing)
        // or do a short extension and collection
        if (input.getGamepadTracker1().isFirstFrameRightTrigger())

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
        if (input.getGamepadTracker1().isFirstFrameLeftTrigger())
            collectingSystemManager.tryEnterState(CollectingSystem.StateType.RETRACTING);

        // force spit in case block gets stuck - spits as long as gamepad up is pressed
        if (input.getGamepadTracker1().isDpadUpPressed()) {
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.SPITTING_TEMP);
            ((SpitTempState) collectorManager.getState(Collector.StateType.SPITTING_TEMP)).continueRunning();
        }

        // force collect in case block is imperfectly collected - collects as long as gamepad down is pressed
        if (input.getGamepadTracker1().isDpadDownPressed()) {
            robot.getCollector().getStateManager().tryEnterState(Collector.StateType.COLLECTING_TEMP);
            ((CollectTempState) collectorManager.getState(Collector.StateType.COLLECTING_TEMP)).continueRunning();
        }
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
        if(input.getGamepadTracker2().isRightBumperPressed())
            robot.setIsHighRam(true);
        // TODO: enable low ram setting input once figure out how to ram on low bar
        //if(input.getGamepadTracker2().isRightTriggerPressed())
            //robot.setIsHighRam(false);

        // a = button for state progression
        if(input.getGamepadTracker1().isFirstFrameA())
            switch(robot.getLiftingSystem().getStateManager().getActiveStateType()) {

                case TROUGH:
                    // activating manual transfer
                    if(!robot.canTransfer() && robot.getCollector().hasValidBlockColor())
                        robot.setCanTransfer(true); // I set to true so that next frame the execute in TroughState will lower lift and actually transfer block
                    else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
                        // activating transition to deposit in basket
                        if(robot.isDepositing() && robot.getGrabber().hasBlock())
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_BASKET);
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
                    if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_BEFORE)
                        robot.getLift().getTransitionState().setGoalState(robot.getLift().getRamAfterPos(), Lift.StateType.RAM_AFTER);
                    // resetting lifting system
                    else if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.RAM_AFTER)
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
                    else if (robot.getGrabber().getTransitionState().getGoalStatePosition() == Grabber.OPEN_POS)
                        robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
                    break;

                // allows for resetting of lifting system to trough during transition from trough
                case TROUGH_TO_DROP_AREA:
                    robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.DROP_AREA_TO_TROUGH);
                    break;
            }
    }
    private void listenForHangingInput() {
        //temp code
        //if (input.getGamepadTracker1().isBPressed() && robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.FULL_DOWN)
        //    robot.getHanger().getStateManager().tryEnterState(Hanger.StateType.GOING_UP);

        // automatically start raising hanging at 2:20
        //if (timeSinceStart >= 140 && robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.FULL_DOWN)
        //    robot.getHanger().getStateManager().tryEnterState(Hanger.StateType.GOING_UP);
        // lower hanging to raise robot
        if (input.getGamepadTracker2().isFirstFrameX())
            if(robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.FULL_DOWN)
                robot.getHanger().getTransitionState().setGoalState(Hanger.UP_TICK, Hanger.StateType.UP);
            else if(robot.getHanger().getStateManager().getActiveStateType() == Hanger.StateType.UP)
                robot.getHanger().getTransitionState().setGoalState(Hanger.HANG_DOWN_TICK, Hanger.StateType.HANG_DOWN);
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
