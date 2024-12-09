package org.firstinspires.ftc.teamcode.robot;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robotStates.NothingState;
import org.firstinspires.ftc.teamcode.robotStates.liftingSystem.*;
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.Helper;

public class LiftingSystem {
    private final BrainSTEMRobot robot;
    public enum StateType {
        TROUGH, TRANSFER, KNOCK_BLOCK, // trough states
        TROUGH_TO_BASKET, BASKET_TO_BASKET, BASKET_DEPOSIT, BASKET_TO_DROP_AREA, // depositing block in basket
        TROUGH_TO_DROP_AREA, DROP_AREA, DROP_AREA_TO_TROUGH, DROP_AREA_TO_RAM, RAM_TO_DROP_AREA, RAM_TO_RAM, SPECIMEN_RAM, RAM_TO_TROUGH // ramming specimen on bar
    }
    public static final Vector2d DEPOSIT_SAFETY_POS = new Vector2d(-48, -48);
    public static final Vector2d DEPOSIT_CORNER = new Vector2d(-72, -72);
    public static final double DEPOSIT_SAFETY_DIST = Helper.dist(DEPOSIT_SAFETY_POS, DEPOSIT_CORNER);
    private boolean buttonACued; // if a is cued during transition, an action should automatically occur once transition is done
    private boolean stayInTrough;
    private final StateManager<StateType> stateManager;

    public LiftingSystem(BrainSTEMRobot robot) {
        this.robot = robot;

        stateManager = new StateManager<>(StateType.TROUGH);

        stateManager.addState(StateType.TROUGH, new TroughState());
        stateManager.addState(StateType.TRANSFER, new TransferState());
        stateManager.addState(StateType.KNOCK_BLOCK, new KnockBlockState());

        stateManager.addState(StateType.TROUGH_TO_BASKET, new TroughToBasketState());
        stateManager.addState(StateType.BASKET_TO_BASKET, new BasketToBasketState());
        stateManager.addState(StateType.BASKET_DEPOSIT, new NothingState<>(StateType.BASKET_DEPOSIT));
        stateManager.addState(StateType.BASKET_TO_DROP_AREA, new BasketToDropAreaState());

        stateManager.addState(StateType.TROUGH_TO_DROP_AREA, new TroughToDropAreaState());
        stateManager.addState(StateType.DROP_AREA, new DropAreaState());
        stateManager.addState(StateType.DROP_AREA_TO_TROUGH, new DropAreaToTroughState());
        stateManager.addState(StateType.DROP_AREA_TO_RAM, new DropAreaToRamState());
        stateManager.addState(StateType.SPECIMEN_RAM, new NothingState<>(StateType.SPECIMEN_RAM));
        stateManager.addState(StateType.RAM_TO_TROUGH, new RamToTroughState()); // this resetting also handles the actual ram (bc you want to reset automatically right after ram is done)

        stateManager.setupStates(robot, stateManager);

        buttonACued = false;
        stayInTrough = true;
    }

    public void update(double dt) {
        stateManager.update(dt);
    }
    public BrainSTEMRobot getRobot() {
        return robot;
    }

    public StateManager<StateType> getStateManager() {
        return stateManager;
    }
    public boolean getButtonACued() {
        return buttonACued;
    }
    public void setButtonACued(boolean buttonACued) {
        this.buttonACued = buttonACued;
    }
    public boolean getStayInTrough() {
        return stayInTrough;
    }
    public void setStayInTrough(boolean stayInTrough) {
        this.stayInTrough = stayInTrough;
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
                    Subsystem.setMotorPower(robot.getLift().getLiftMotor(), robot.getLift().getPid().update(robot.getLift().getLiftMotor().getCurrentPosition()));
                // moving lift up once pid set to move back up
                else if(robot.getLift().getPid().getTarget() == Lift.TROUGH_SAFETY_POS)
                    Subsystem.setMotorPower(robot.getLift().getLiftMotor(), robot.getLift().getPid().update(robot.getLift().getLiftMotor().getCurrentPosition()));
            }

            return !(robot.getCollector().getBlockColorSensor().getRawBlockColor() == BlockColor.NONE
            && Subsystem.inRange(robot.getLift().getLiftMotor(), Lift.TROUGH_SAFETY_POS, Lift.AUTO_DESTINATION_THRESHOLD));
        };
    }
    public Action transferBlockOnce() {
        return new SequentialAction(
                robot.getLift().moveTo(Lift.AUTO_TROUGH_POS),
                robot.getGrabber().close(),
                robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS)
        );
    }

    public Action transferToDropOff() {
        return new SequentialAction(
                robot.getArm().rotateTo(Arm.DROP_OFF_POS, Arm.TRANSFER_TO_DROP_AREA_TIME),
                robot.getLift().moveTo(Lift.DROP_AREA_POS)
        );
    }
    public Action depositHighInitial() {
        return new SequentialAction(
            robot.getLift().moveTo(Lift.HIGH_BASKET_POS, Lift.HIGH_BASKET_SAFETY_POS),
            new ParallelAction(
                    robot.getArm().rotateTo(Arm.BASKET_DROP_POS, 0),
                    new SequentialAction(
                            new SleepAction(Arm.BASKET_SAFETY_TO_BASKET_DROP_TIME - Grabber.FULL_ROTATION_TIME/2),
                            robot.getGrabber().open()
                    )
            )
        );
    }
    public Action depositHigh() {
        return new SequentialAction(
                robot.getArm().rotateTo(Arm.BASKET_SAFETY_POS, Arm.TRANSFER_TO_BASKET_SAFETY_TIME),
                depositHighInitial()
        );
    }
    public Action lowerFromDeposit() {
        return new SequentialAction(
                robot.getArm().rotateTo(Arm.BASKET_SAFETY_POS, Arm.BASKET_SAFETY_TO_BASKET_DROP_TIME),
                new ParallelAction(
                        robot.getLift().moveToWithoutPid(Lift.TROUGH_SAFETY_POS),
                        robot.getArm().rotateTo(Arm.TRANSFER_POS, Arm.TRANSFER_TO_BASKET_SAFETY_TIME)
                )
        );
    }

    // specimen actions
    public Action setupHighSpecimenRam() {
        return new SequentialAction(
                robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS),
                new ParallelAction(
                    robot.getLift().moveTo(Lift.HIGH_RAM_BEFORE_POS),
                    robot.getArm().rotateTo(Arm.SPECIMEN_HANG_POS, Arm.UP_TO_TRANSFER_TIME + Arm.SPECIMEN_HANG_TO_UP_TIME)
                )
        );
    }
    public Action ramHighSpecimen() {
        return new SequentialAction(
                robot.getLift().moveTo(Lift.HIGH_RAM_AFTER_POS),
                robot.getGrabber().open()
        );
    }
    public Action resetSpecimenRam() {
        return new SequentialAction(
            robot.getArm().rotateTo(Arm.UP_POS, Arm.SPECIMEN_HANG_TO_UP_TIME),
            new ParallelAction(
                robot.getArm().rotateTo(Arm.TRANSFER_POS, Arm.UP_TO_TRANSFER_TIME),
                robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS))
        );
    }
}
