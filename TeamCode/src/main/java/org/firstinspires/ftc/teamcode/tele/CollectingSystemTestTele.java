package org.firstinspires.ftc.teamcode.tele;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;

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
import org.firstinspires.ftc.teamcode.stateMachine.StateManager;
import org.firstinspires.ftc.teamcode.util.GamepadTracker;
import org.firstinspires.ftc.teamcode.util.Input;
import org.firstinspires.ftc.teamcode.util.MotorPowerJamTracker;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CollectingSystemTestTele")
public class CollectingSystemTestTele extends LinearOpMode {
    /*
    controls
    left and right bumpers: move extension in and out - cannot go beyong params.maxExtensionTick and 0
    left and right triggers: hinge up and down - goes to hingeUpPosition and hingeDownPosition
    a and b - collect and collect temp - uses collectPower and collectTempPower to determine spindle motor power
    x and y - spit and spit temp - uses spitPower and spitTempPower
    */

    public static class Params {
        public int maxExtensionTick = 1880;
        public double extensionSearchPower = 0.55;

        public int spindleJamFrames = 4; // number of frames it gets to try to unjam
        public int spindleJamRequiredEncoderTicks = 10; // must travel at least 10 ticks per frame to unjam
        public double spindleJamPower = -0.2; // power given to spindle motor when it is jammed
        public double collectPower = 1;
        public double spitPower = -1;
        public double spitTempPower = -0.5;
        public double collectTempPower = 0.4;

        public int maxHingeTick = 2150;
        public int minHingeTick = 1480;
        public double hingeUpPosition = 0.99;
        public double hingeDownPosition = 0.01;
    }
    public static CollectingSystemTestingTele.Params params = new CollectingSystemTestingTele.Params();

    private Input input;
    private DcMotorEx extensionMotor, spindleMotor;
    private ServoImplEx hingeServo;

    private MotorPowerJamTracker spindleJamTracker;
    private final Pose2d BEGIN_POSE = new Pose2d(-24, 0, 0);
    @Override
    public void runOpMode() throws InterruptedException {
        input = new Input(gamepad1, gamepad2);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // update custom input
            input.update();
        }
    }
}
