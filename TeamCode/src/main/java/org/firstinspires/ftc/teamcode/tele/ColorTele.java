package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.BlockColorSensor;
import org.firstinspires.ftc.teamcode.util.Input;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleBlockColor")
public class ColorTele extends LinearOpMode {
    private Input input;

    @Override
    public void runOpMode() throws InterruptedException {
        input = new Input(gamepad1, gamepad2);

        BlockColorSensor blockColorSensor = new BlockColorSensor(hardwareMap);
        BlockColor curBlockColor = BlockColor.NONE;

        boolean collectingData = false;

        int maxR = -1;
        int minR = 10000000;
        int maxG = -1;
        int minG = 10000000;
        int maxB = -1;
        int minB = 10000000;

        waitForStart();

        while (opModeIsActive()) {
            input.update();

            int r = blockColorSensor.getColorSensor().red();
            int g = blockColorSensor.getColorSensor().green();
            int b = blockColorSensor.getColorSensor().blue();
            double sum = r + g + b;

            if(collectingData) {
                // updating border values
                if (r > maxR)
                    maxR = r;
                if (r < minR)
                    minR = r;

                if (g > maxG)
                    maxG = g;
                if (g < minG)
                    minG = g;

                if (b > maxB)
                    maxB = b;
                if (b < minB)
                    minB = b;
            }

            // toggling collection mode
            if(input.getGamepadTracker1().isFirstFrameA())
                collectingData = !collectingData;

            // resetting color values
            if(!collectingData) {
                maxR = -1;
                minR = 10000000;
                maxG = -1;
                minG = 10000000;
                maxB = -1;
                minB = 10000000;
            }


            telemetry.addData("controls", "");
            telemetry.addData("  a", "toggle data collection");

            telemetry.addData("collectingData", collectingData);
            telemetry.addData("", "");
            telemetry.addData("current color values", "");
            telemetry.addData("  r percent", r / sum);
            telemetry.addData("  g percent", g / sum);
            telemetry.addData("  b percent", b / sum);

            telemetry.addData("", "");
            telemetry.addData("max r, min r", maxR + ", " + minR);
            telemetry.addData("max g, min g", maxG + ", " + minG);
            telemetry.addData("max b, min b", maxB + ", " + minB);

            telemetry.update();
        }
    }
}
