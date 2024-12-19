package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.util.Input;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LimeLightTele")
public class LimeLightTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Input input = new Input(gamepad1, gamepad2);
        telemetry.setMsTransmissionInterval(11);

        int currentPipeline = 0;
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(currentPipeline);
        limelight.start();

        while (opModeIsActive()) {
            input.update();

            // listening for toggling input
            if(input.getGamepadTracker1().isFirstFrameA()) {
                currentPipeline++;
                currentPipeline %= 4;
            }

            telemetry.addData("current pipeline:", currentPipeline);

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    telemetry.addData("target x", result.getTx());
                    telemetry.addData("target y", result.getTy());
                }
            }
            else
                telemetry.addData("no target found", "");

            telemetry.addData("", "");
            telemetry.addData("controls", "");
            telemetry.addData("  a", "toggle current pipeline");
            telemetry.update();
        }
    }
}