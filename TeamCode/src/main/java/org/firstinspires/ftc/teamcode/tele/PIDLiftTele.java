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
import org.firstinspires.ftc.teamcode.util.PIDController;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "PIDLiftTele")
public class PIDLiftTele extends LinearOpMode {

    private Input input;
    private BrainSTEMRobot robot;

    private double timeSinceStart;

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
        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, AllianceColor.RED);
        PIDController pid = robot.getLift().getPid();

        int editMode = 0; // 0 = kA, 1 = kP, etc


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


            // checking for switching editMode
            if(input.getGamepadTracker2().isFirstFrameA())
                editMode += 1;
            editMode %= 4;


            // checking for adjusting k values
            if(input.getGamepadTracker2().isFirstFrameDpadUp())
                incrementPIDVar(pid, editMode, 0.05);
            else if(input.getGamepadTracker2().isFirstFrameDpadDown())
                incrementPIDVar(pid, editMode, -0.05);

            // checking for pid mode toggling
            if(input.getGamepadTracker1().isFirstFrameX())
                robot.setInPidMode(!robot.getInPidMode());


            listenForLiftingInput();

            robot.update(dt);
            telemetry.addData("error", pid.getTarget() - robot.getExtension().getExtensionMotor().getCurrentPosition());
            telemetry.addData("motor power", robot.getLift().getLiftMotor().getPower());
            telemetry.addData("lift encoder", robot.getLift().getLiftMotor().getCurrentPosition());
            telemetry.addData("collecting system state, ", robot.getCollectingSystem().getStateManager().getActiveStateType());
            telemetry.addData("a", gamepad2.a + " | " + input.getGamepadTracker2().isAPressed());
            telemetry.addData("editMode", editMode);
            telemetry.addData("kP", pid.getkP());
            telemetry.addData("kI", pid.getkI());
            telemetry.addData("kD", pid.getkD());
            telemetry.addData("target", pid.getTarget());
            telemetry.addData("extension state", robot.getExtension().getStateManager().getActiveStateType());
            telemetry.update();
        }
    }

    private void listenForLiftingInput() {
        if(input.getGamepadTracker1().isFirstFrameX())
            robot.setInDepositingMode(!robot.getInDepositingMode());
        // checking changes in basket/bar heights
        if(input.getGamepadTracker2().isLeftBumperPressed()) {
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_BASKET);
            robot.setIsHighDeposit(true);
        }

        if(input.getGamepadTracker2().isLeftTriggerPressed()) {
            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.BASKET_TO_BASKET);
            robot.setIsHighDeposit(false);
        }

        // button for "progressing" a state
        if(input.getGamepadTracker1().isFirstFrameA())
            switch(robot.getLiftingSystem().getStateManager().getActiveStateType()) {
                case TROUGH:
                    if(robot.getLift().getStateManager().getActiveStateType() == Lift.StateType.TROUGH_SAFETY)
                        if(robot.getInDepositingMode())
                        //if(robot.getBlockColorHeld() == BlockColor.YELLOW)
                            robot.getLiftingSystem().getStateManager().tryEnterState(LiftingSystem.StateType.TROUGH_TO_BASKET);
                        else
                        //else if(robot.getBlockColorHeld() == robot.getColorFromAlliance())
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

}
