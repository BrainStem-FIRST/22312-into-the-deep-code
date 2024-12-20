package org.firstinspires.ftc.teamcode.tele;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.LiftingSystem;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.GamepadTracker;
import org.firstinspires.ftc.teamcode.util.Input;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class TeleMain extends LinearOpMode {
    public static class Params {
        public AllianceColor allianceColor = AllianceColor.RED;
    }
    public static Params PARAMS = new Params();
    private BrainSTEMRobot robot;
    private final Pose2d BEGIN_POSE = new Pose2d(-24, -7.5, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, PARAMS.allianceColor, BEGIN_POSE, new Input(gamepad1, gamepad2));

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

            robot.addTelemetry();
            robot.getDriveTrain().addTelemetry(telemetry);
            robot.getLiftingSystem().addTelemetry(telemetry);

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

            robot.getHanger().addTelemetry(telemetry);

            telemetry.update();
        }
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
}
