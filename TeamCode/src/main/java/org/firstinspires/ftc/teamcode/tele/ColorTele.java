package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BlockColorSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleBlockColor")
public class ColorTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BlockColorSensor blockColorSensor = new BlockColorSensor(hardwareMap, telemetry);

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

            telemetry.addData("sensor red value", blockColorSensor.red());
            telemetry.addData("sensor green value", blockColorSensor.green());
            telemetry.addData("sensor blue value", blockColorSensor.blue());

            telemetry.addData("ideal blue block r percent", BlockColorSensor.RED_BLOCK_PERCENTS[0]);
            telemetry.addData("ideal blue block g percent", BlockColorSensor.RED_BLOCK_PERCENTS[1]);
            telemetry.addData("ideal blue block b percent", BlockColorSensor.RED_BLOCK_PERCENTS[2]);

            telemetry.addData("has red color", blockColorSensor.hasColor(BlockColorSensor.BlockColor.RED));
            telemetry.addData("has blue color", blockColorSensor.hasColor(BlockColorSensor.BlockColor.BLUE));
            telemetry.addData("has yellow color", blockColorSensor.hasColor(BlockColorSensor.BlockColor.YELLOW));
            telemetry.update();
        }
    }
}