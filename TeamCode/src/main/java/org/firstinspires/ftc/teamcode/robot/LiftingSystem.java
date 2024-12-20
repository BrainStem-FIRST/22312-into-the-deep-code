package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.*;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;

public class LiftingSystem {
    private final BrainSTEMRobot robot;
    public enum StateType {
        TROUGH, TRANSFER, KNOCK_BLOCK,
        TROUGH_TO_BASKET, BASKET_TO_BASKET, BASKET_DEPOSIT, BASKET_TO_DROP_AREA, // depositing block in basket
        TROUGH_TO_DROP_AREA, DROP_AREA, DROP_AREA_TO_TROUGH, DROP_AREA_TO_RAM, RAM_TO_DROP_AREA, SPECIMEN_RAM // ramming specimen on bar
    }
    private final StateManager<StateType> stateManager;

    public LiftingSystem(BrainSTEMRobot robot) {
        this.robot = robot;

        stateManager = new StateManager<>(StateType.TROUGH);

        stateManager.addState(StateType.TROUGH, new TroughState());
        stateManager.addState(StateType.TRANSFER, new TransferState());
        stateManager.addState(StateType.KNOCK_BLOCK, new KnockBlockState());
        stateManager.addState(StateType.TROUGH_TO_BASKET, new TroughToBasketState());
        stateManager.addState(StateType.BASKET_TO_BASKET, new BasketToBasketState());
        stateManager.addState(StateType.BASKET_DEPOSIT, new BasketDepositState());
        stateManager.addState(StateType.BASKET_TO_DROP_AREA, new BasketToDropAreaState());
        stateManager.addState(StateType.TROUGH_TO_DROP_AREA, new TroughToDropAreaState());
        stateManager.addState(StateType.DROP_AREA, new DropAreaState());
        stateManager.addState(StateType.DROP_AREA_TO_TROUGH, new DropAreaToTroughState());
        stateManager.addState(StateType.DROP_AREA_TO_RAM, new DropAreaToRamState());
        stateManager.addState(StateType.SPECIMEN_RAM, new SpecimenRamState());
        stateManager.addState(StateType.RAM_TO_DROP_AREA, new RamToDropAreaState());

        stateManager.setupStates(robot, stateManager);

    }

    public void update(double dt) {
        stateManager.update(dt);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("", "");
        telemetry.addData("is high basket", robot.isHighDeposit());
        telemetry.addData("depositing mode", robot.isDepositing());
        telemetry.addData("lifting system", robot.getLiftingSystem().getStateManager().getActiveState().toString());
        telemetry.addData("lift", robot.getLift().getStateManager().getActiveState().toString());
        telemetry.addData("arm", robot.getArm().getStateManager().getActiveStateType());
        telemetry.addData("grabber", robot.getGrabber().getStateManager().getActiveStateType());
        telemetry.addData("  grabber has specimen", robot.getGrabber().hasSpecimen());
        telemetry.addData("  grabber block color", robot.getGrabber().getBlockColorHeld());
    }

    public BrainSTEMRobot getRobot() {
        return robot;
    }
    public StateManager<StateType> getStateManager() {
        return stateManager;
    }

    // continuous block transfer until block is grabbed onto (also uses pid)
    public Action transferBlock() {
        ElapsedTime grabTimer = new ElapsedTime();
        return telemetryPacket -> {
            if(robot.getCollector().getBlockColorSensor().getRawBlockColor() != BlockColor.NONE) {
                // setting target to try and transfer
                if(Subsystem.inRange(robot.getLift().getLiftMotor(), Lift.TROUGH_SAFETY_POS, Lift.AUTO_DESTINATION_THRESHOLD)) {
                    robot.getLift().getPid().setTarget(Lift.AUTO_TROUGH_POS);
                    robot.getGrabber().getGrabServo().setPosition(Grabber.OPEN_POS);
                }
                // checking what to do once lift gets low enough to grab onto block
                else if(Subsystem.inRange(robot.getLift().getLiftMotor(), Lift.AUTO_TROUGH_POS, Lift.AUTO_DESTINATION_THRESHOLD))
                    // closes onto block if haven't already
                    if(robot.getGrabber().getGrabServo().getPosition() == Grabber.OPEN_POS) {
                        robot.getGrabber().getGrabServo().setPosition(Grabber.CLOSE_POS);
                        grabTimer.reset();
                    }
                    // raises lift after set time passed (to ensure grabber has block)
                    else if(grabTimer.seconds() >= Grabber.FULL_ROTATION_TIME)
                        robot.getLift().getPid().setTarget(Lift.TROUGH_SAFETY_POS);

                // moving lift down once pid is set to transfer down
                if(robot.getLift().getPid().getTarget() == Lift.AUTO_TROUGH_POS)
                    robot.getLift().setLiftPower(robot.getLift().getPid().update(robot.getLift().getLiftMotor().getCurrentPosition()));
                // moving lift up once pid set to move back up
                else if(robot.getLift().getPid().getTarget() == Lift.TROUGH_SAFETY_POS)
                    robot.getLift().setLiftPower(robot.getLift().getPid().update(robot.getLift().getLiftMotor().getCurrentPosition()));
            }

            return !(robot.getCollector().getBlockColorSensor().getRawBlockColor() == BlockColor.NONE
            && Subsystem.inRange(robot.getLift().getLiftMotor(), Lift.TROUGH_SAFETY_POS, Lift.AUTO_DESTINATION_THRESHOLD));
        };
    }
    public Action transferBlockOnce() {
        return new SequentialAction(
                robot.getLift().moveTo(Lift.AUTO_TROUGH_POS, Lift.MEDIUM_TRANSITION_KP, Lift.SMALL_TRANSITION_KI),
                robot.getGrabber().close(),
                robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS, Lift.MEDIUM_TRANSITION_KP * 1.2, Lift.ZERO_KI)
        );
    }
    public Action depositHighInitial() {
        return new SequentialAction(
            robot.getLift().moveTo(Lift.HIGH_BASKET_POS, Lift.BIG_TRANSITION_KP, Lift.SMALL_TRANSITION_KI),
            robot.getArm().rotateTo(Arm.BASKET_DROP_POS, Arm.BASKET_SAFETY_TO_BASKET_DROP_TIME),
            robot.getGrabber().open()
        );
    }
    public Action depositHigh() {
        return new SequentialAction(
                robot.getArm().rotateTo(Arm.BASKET_SAFETY_POS, Arm.TRANSFER_TO_BASKET_SAFETY_TIME),
                depositHighInitial()
        );
    }
    public Action lowerFromDeposit() {
        return new ParallelAction(
                robot.getLift().moveToTime(Lift.AUTO_TROUGH_SAFETY_POS, 2.7, Lift.MEDIUM_TRANSITION_KP * 0.9, Lift.ZERO_KI),
                robot.getArm().rotateTo(Arm.TRANSFER_POS, Arm.TRANSFER_TO_BASKET_SAFETY_TIME)
        );
    }

    public Action setupHighSpecimenRam() {
        robot.telemetry.addData("in normal setup ram", "");
        robot.telemetry.update();
        return new SequentialAction(
                robot.getGrabber().close(),
                new ParallelAction(
                        robot.getLift().moveToTime(Lift.HIGH_RAM_BEFORE_POS, 1.5, Lift.MEDIUM_TRANSITION_KP, Lift.ZERO_KI),
                        robot.getArm().rotateTo(Arm.SPECIMEN_HANG_POS, 0)
                )
        );

    }
    public Action ramHighSpecimen() {
        robot.telemetry.addData("ramming specimen", "");
        robot.telemetry.update();
        return new SequentialAction(
                robot.getLift().moveToTime(Lift.HIGH_RAM_AFTER_POS + 80, 2, Lift.BIG_TRANSITION_KP, Lift.ZERO_KI),
                robot.getGrabber().open()
        );
    }
    public Action resetSpecimenRam() {
        robot.telemetry.addData("resetting after ramming", "");
        robot.telemetry.update();
        return new ParallelAction(
                robot.getArm().rotateTo(Arm.DROP_OFF_POS, Arm.DROP_AREA_TO_RAM_TIME),
                robot.getLift().moveTo(Lift.DROP_AREA_POS)
        );
    }
}
