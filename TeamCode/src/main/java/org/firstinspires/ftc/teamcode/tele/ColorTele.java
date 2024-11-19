package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.BlockColorSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleBlockColor")
public class ColorTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

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

            int r = blockColorSensor.red();
            int g = blockColorSensor.green();
            int b = blockColorSensor.blue();
            double sum = r + g + b;

            // checking switch of block color input
            if(gamepad1.b)
                curBlockColor = BlockColor.BLUE;
            else if(gamepad1.a)
                curBlockColor = BlockColor.RED;
            else if(gamepad1.y)
                curBlockColor = BlockColor.YELLOW;
            else if(gamepad1.x)
                curBlockColor = BlockColor.NONE;

            // checking switch of dataMode input
            if(gamepad1.dpad_down)
                blockColorSensor.setDataMode(false);
            else if(gamepad1.dpad_up)
                blockColorSensor.setDataMode(true);

            telemetry.addData("collecting data", blockColorSensor.getDataMode());


            telemetry.addData("", blockColorSensor.update(curBlockColor));


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
