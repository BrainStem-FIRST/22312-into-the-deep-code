package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.GamepadTracker;
import org.firstinspires.ftc.teamcode.util.Input;
import org.firstinspires.ftc.teamcode.util.MotorCurrentTracker;

import java.io.FileNotFoundException;
import java.io.PrintWriter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CollectingSystemTestingTele")
@Config
public class CollectingSystemTestingTele extends LinearOpMode {

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

        public double collectPower = 0.99;
        public double spitPower = -0.99;
        public double spitTempPower = -0.5;
        public double collectTempPower = 0.4;

        public int maxHingeTick = 2150;
        public int minHingeTick = 1480;
        public double hingeUpPosition = 0.99;
        public double hingeDownPosition = 0.01;


    }
    public static Params params = new Params();

    private DcMotorEx extensionMotor, spindleMotor;
    private MotorCurrentTracker spindleCurrentTracker;
    private ServoImplEx hingeServo;
    private boolean collectingData;
    private String collectionFilePath = "spindleData.txt";
    private PrintWriter printWriter;

    @Override
    public void runOpMode() throws InterruptedException {
        Input input = new Input(gamepad1, gamepad2);
        try {
            printWriter = new PrintWriter(collectionFilePath);
        } catch (FileNotFoundException e) {
            throw new RuntimeException(e);
        }

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();

        waitForStart();

        extensionMotor = hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spindleMotor = hardwareMap.get(DcMotorEx.class, "CollectSpindleMotor");
        spindleCurrentTracker = new MotorCurrentTracker(spindleMotor, Collector.TELE_JAM_CURRENT_THRESHOLD, Collector.TELE_JAM_VALIDATION_FRAMES, Collector.TELE_JAM_SAFETY_FRAMES);

        hingeServo = hardwareMap.get(ServoImplEx.class, "CollectHingeServo");
        hingeServo.setPwmRange(new PwmControl.PwmRange(params.minHingeTick, params.maxHingeTick));

        while(opModeIsActive()) {
            input.update();
            spindleCurrentTracker.updateCurrentTracking();

            listenForExtensionControls(input.getGamepadTracker1());
            listenForSpindleControls(input.getGamepadTracker1());
            listenForHingeControls(input.getGamepadTracker1());

            telemetry.addData("extension encoder", extensionMotor.getCurrentPosition());
            telemetry.addData("extension motor power", extensionMotor.getPower());
            telemetry.addData("manual pid value", Range.clip(0.01 * (Extension.MIN_POSITION - extensionMotor.getCurrentPosition()), -1, 1));

            telemetry.addData("", "");
            telemetry.addData("collector motor current", spindleMotor.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("collector motor power", spindleMotor.getPower());
            telemetry.addData("is abnormal validated", spindleCurrentTracker.hasValidatedAbnormalCurrent());
            telemetry.addData("abnormal frames", spindleCurrentTracker.getConsecutiveAbnormalFrames());
            telemetry.addData("is abnormal raw", spindleCurrentTracker.hasRawAbnormalCurrent());
            telemetry.addData("collecting collector data", collectingData);

            telemetry.addData("", "");
            telemetry.addData("controls", "");
            telemetry.addData("  x", "togge collecting mode");

            telemetry.update();
        }
    }

    public void listenForExtensionControls(GamepadTracker gamepadTracker) {
        if (gamepadTracker.isRightBumperPressed() && extensionMotor.getCurrentPosition() < params.maxExtensionTick)
                Subsystem.setMotorPower(extensionMotor, params.extensionSearchPower);
        else if (gamepadTracker.isLeftBumperPressed() && extensionMotor.getCurrentPosition() > 0)
                Subsystem.setMotorPower(extensionMotor, -params.extensionSearchPower);
        else
            Subsystem.setMotorPower(extensionMotor, 0);
    }
    public void listenForSpindleControls(GamepadTracker gamepadTracker) {
        // checking for actual collecting (with spitting check implemented)
        if (spindleCurrentTracker.hasValidatedAbnormalCurrent()) {
            Subsystem.setMotorPower(spindleMotor, Collector.SPIT_TEMP_POWER);
        }
        else {
            if (gamepadTracker.isAPressed())
                Subsystem.setMotorPower(spindleMotor, Collector.TELE_COLLECT_POWER);
            else
                Subsystem.setMotorPower(spindleMotor, 0);
        }

        // checking for when to write data into file
        if(gamepadTracker.isFirstFrameX()) {
            collectingData = !collectingData;

            if(!collectingData) {
                printWriter.close();
                telemetry.addData("file successfully closed", "");
            }
        }

        // actually writing to file
        if(collectingData) {
            telemetry.addData("writing to file", "");
            // prints in format: power current hasRawAbnormal hasValidatedAbnormal
            printWriter.print(spindleMotor.getPower() + " ");
            printWriter.print(spindleMotor.getCurrent( CurrentUnit.MILLIAMPS) + " ");
            printWriter.print(spindleCurrentTracker.hasRawAbnormalCurrent() + " ");
            printWriter.println(spindleCurrentTracker.hasValidatedAbnormalCurrent());
        }

    }
    public void listenForHingeControls(GamepadTracker gamepadTracker) {
        if (gamepadTracker.isRightTriggerPressed())
            hingeServo.setPosition(params.hingeDownPosition);
        else if (gamepadTracker.isLeftTriggerPressed())
            hingeServo.setPosition(params.hingeUpPosition);
    }
}

