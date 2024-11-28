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

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();


        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;

        while (opModeIsActive()) {
            // update dt
            prevTime = currentTime;
            currentTime = System.currentTimeMillis();
            dt = (currentTime - prevTime) / 1000.;

            input.update();

            int r = blockColorSensor.red();
            int g = blockColorSensor.green();
            int b = blockColorSensor.blue();
            double sum = r + g + b;

            // checking switch of block color input
            if(input.getGamepadTracker1().isFirstFrameB())
                curBlockColor = BlockColor.RED;
            else if(input.getGamepadTracker1().isFirstFrameX())
                curBlockColor = BlockColor.BLUE;
            else if(input.getGamepadTracker1().isFirstFrameY())
                curBlockColor = BlockColor.YELLOW;
            else if(input.getGamepadTracker1().isFirstFrameA())
                curBlockColor = BlockColor.NONE;

            // checking switch of dataMode input
            if(input.getGamepadTracker1().isFirstFrameDpadDown())
                blockColorSensor.setDataMode(!blockColorSensor.getDataMode());

            telemetry.addData("collecting data", blockColorSensor.getDataMode());
            telemetry.addData("manually selected block", curBlockColor);

            telemetry.addData("", blockColorSensor.updateBlockColorTesting(curBlockColor));

            telemetry.addData("r percent", r / sum);
            telemetry.addData("g percent", g / sum);
            telemetry.addData("b percent", b / sum);
            telemetry.addData("has red block", blockColorSensor.hasColor(BlockColor.RED));
            telemetry.addData("has yellow block", blockColorSensor.hasColor(BlockColor.YELLOW));
            telemetry.addData("has blue block", blockColorSensor.hasColor(BlockColor.BLUE));
            telemetry.update();
        }
    }
}
