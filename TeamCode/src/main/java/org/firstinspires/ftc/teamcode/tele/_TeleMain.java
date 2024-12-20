package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.Input;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class _TeleMain extends LinearOpMode {
    public static AllianceColor allianceColor = AllianceColor.RED;
    public static boolean sampleMode = false;
    public static Pose2d beginPose = new Pose2d(-24, -7.5, Math.toRadians(90));
    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, allianceColor, beginPose, new Input(gamepad1, gamepad2));

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();

        waitForStart();

        robot.getStateManager().tryEnterState(BrainSTEMRobot.StateType.SETTING_UP); // setup robot

        long currentAbsoluteTime = System.currentTimeMillis();
        long prevAbsoluteTime;
        double dt;
        double maxDt = 0;
        double minDt = 10;
        double currentGameTime = 0;

        while (opModeIsActive()) {

            // update dt
            prevAbsoluteTime = currentAbsoluteTime;
            currentAbsoluteTime = System.currentTimeMillis();
            dt = (currentAbsoluteTime - prevAbsoluteTime) / 1000.;
            currentGameTime += dt;

            // update robot
            robot.update(dt);

            // for debugging: checking dt
            if(dt > maxDt)
                maxDt = dt;
            if(dt < minDt)
                minDt = dt;

            // for debugging
            telemetry.addData("", "");
            telemetry.addData("game time", currentGameTime);
            telemetry.addData("dt", dt);
            telemetry.addData("max dt", maxDt);
            telemetry.addData("min dt", minDt);

            robot.addTelemetry();
            robot.getDriveTrain().addTelemetry(telemetry);
            robot.getLiftingSystem().addTelemetry(telemetry);
            robot.getCollectingSystem().addTelemetry(telemetry);
            robot.getHanger().addTelemetry(telemetry);

            telemetry.update();
        }
    }
}
