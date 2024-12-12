package org.firstinspires.ftc.teamcode.robotStates.robot;

import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Hanger;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.robotStates.RobotState;

public class SettingUpState extends RobotState<BrainSTEMRobot.StateType> {

    private boolean done;
    private double startHingeTime;
    public SettingUpState() {
        super(BrainSTEMRobot.StateType.SETTING_UP);
        done = false;
        startHingeTime = 0;
    }

    @Override
    public void execute(double dt) {
        if (isFirstTime()) {
            done = false;
            // setting up servos
            robot.getHinge().setHingeServoPosition(Hinge.HINGE_UP_POSITION);
            robot.getGrabber().getGrabServo().setPosition(Grabber.OPEN_POS);
            //robot.getHanger().getTransitionState().setGoalState(Hanger.UP_TICK, Hanger.StateType.UP);
            startHingeTime = getTime();
        }

        boolean inRange = Subsystem.inRange(robot.getLift().getLiftMotor(), Lift.TROUGH_SAFETY_POS, Lift.DESTINATION_THRESHOLD);

        if (!inRange)
            Subsystem.setMotorPosition(robot.getLift().getLiftMotor(), Lift.TROUGH_SAFETY_POS); // move lift to safety
        else {
            robot.getArm().getArmServo().setPosition(Arm.TRANSFER_POS); // move arm to transfer position

            // retract when hinge and lift are done
            if (getTime() - startHingeTime > Hinge.HINGE_UP_TIME) {
                robot.getExtension().retractExtensionMotor(); // sets raw power of extension motor

                // end state
                if (robot.getExtension().hitRetractHardStop()) {
                    done = true;
                    robot.getExtension().setTargetPower(0);
                    robot.getExtension().setExtensionMotorPower(0);
                }
            }

        }
    }

    @Override
    public boolean canEnter() {
        return false;
    }

    @Override
    public boolean canBeOverridden() {
        return false;
    }

    @Override
    public boolean isDone() {
        return done;
    }

    @Override
    public BrainSTEMRobot.StateType getNextStateType() {
        return BrainSTEMRobot.StateType.PLAYING;
    }
}
