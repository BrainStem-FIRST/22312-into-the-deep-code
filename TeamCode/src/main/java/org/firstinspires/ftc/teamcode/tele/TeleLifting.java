package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleLifting")
public class TeleLifting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.BLUE);
        telemetry.addData("", robot.getLiftingSystem());
        telemetry.addData("", robot.getLiftingSystem().getStateManager());

        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;

        while (opModeIsActive()) {
            // update dt
            prevTime = currentTime;
            currentTime = System.currentTimeMillis();
            dt = (currentTime - prevTime) / 1000.;


            telemetry.update();
        }
    }
}
