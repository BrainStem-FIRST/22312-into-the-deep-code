package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Input;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CollectingExtensionTele")
public class ExtensionTestTele extends LinearOpMode {
    private final double TURN_AMP = 0.8;

    private Input input;

    private double timeSinceStart;

    @Override
    public void runOpMode() throws InterruptedException {

        input = new Input(gamepad1, gamepad2);
        DcMotorEx extensionMotor = hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extensionMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();

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
            if (input.getGamepadTracker1().isRightBumperPressed())
                extensionMotor.setPower(0.2);
            if (input.getGamepadTracker1().isLeftBumperPressed())
                extensionMotor.setPower(-0.2);


            telemetry.addData("extension encoder", extensionMotor.getCurrentPosition());

            telemetry.update();
        }
    }
}
