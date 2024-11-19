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
import org.firstinspires.ftc.teamcode.util.PIDController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PIDExtensionTele")
public class PIDExtensionTele extends LinearOpMode {
    BrainSTEMRobot robot;
    Input input;

    public void incrementPIDVar(PIDController pid, int editMode, double amount) {
        switch(editMode) {
            //case 0:
            //    pid.setkA(pid.getkA() + amount);
            //    break;
            case 1:
                pid.setkP(pid.getkP() + amount);
                break;
            case 2:
                pid.setkI(pid.getkI() + amount);
                break;
            case 3:
                pid.setkD(pid.getkD() + amount);
                break;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.BLUE);

        int editMode = 0; // 0 = kA, 1 = kP, etc

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();


        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;
        double interval = 1000/60.; // 60 FPS
        double timeSinceLastUpdate = 0;


        while (opModeIsActive()) {
            // update dt
            prevTime = currentTime;
            currentTime = System.currentTimeMillis();
            dt = (currentTime - prevTime) / 1000.;
            timeSinceLastUpdate += dt;

            // update custom input
            input.update();

            // checking for switching editMode
            if(input.getGamepadTracker2().isFirstFrameA())
                editMode += 1;
            editMode %= 4;


            // checking for adjusting k values
            if(input.getGamepadTracker2().isFirstFrameDpadUp())
                incrementPIDVar(robot.getExtension().getPid(), editMode, 0.05);
            else if(input.getGamepadTracker2().isFirstFrameDpadDown())
                incrementPIDVar(robot.getExtension().getPid(), editMode, -0.05);

            // checking for pid mode toggling
            if(input.getGamepadTracker1().isFirstFrameX())
                robot.setInPidMode(!robot.getInPidMode());

            robot.update(dt);
            listenForCollectionInput();
            telemetry.addData("pid mode", robot.getInPidMode());
            telemetry.addData("error", robot.getExtension().getPid().getTarget() - robot.getExtension().getExtensionMotor().getCurrentPosition());
            telemetry.addData("motor power", robot.getExtension().getExtensionMotor().getPower());
            telemetry.addData("extension encoder", robot.getExtension().getExtensionMotor().getCurrentPosition());
            telemetry.addData("collecting system state, ", robot.getCollectingSystem().getStateManager().getActiveStateType());
            telemetry.addData("extension state, ",  robot.getExtension().getStateManager().getActiveStateType());
            telemetry.addData("hinge state", robot.getHinge().getStateManager().getActiveStateType());
            telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            telemetry.addData("a", gamepad2.a + " | " + input.getGamepadTracker2().isAPressed());
            telemetry.addData("editMode", editMode);
            telemetry.addData("kP", robot.getExtension().getPid().getkP());
            telemetry.addData("kI", robot.getExtension().getPid().getkI());
            telemetry.addData("kD", robot.getExtension().getPid().getkD());
            telemetry.addData("target", robot.getExtension().getPid().getTarget());
            telemetry.addData("extension state", robot.getExtension().getStateManager().getActiveStateType());
            telemetry.update();
        }
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
