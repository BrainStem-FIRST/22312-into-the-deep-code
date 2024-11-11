package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.BlockColor;
import org.firstinspires.ftc.teamcode.robot.BlockColorSensor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleBlockColor")
public class ColorTele extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        BlockColorSensor blockColorSensor = new BlockColorSensor(hardwareMap);

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
