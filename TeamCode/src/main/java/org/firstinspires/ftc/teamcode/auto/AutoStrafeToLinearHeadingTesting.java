package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

@Autonomous
@Config
public class AutoStrafeToLinearHeadingTesting extends LinearOpMode {
    public static class Params {
        public double x1 = -48;
        public double y1 = -60;
        public double h1 = 0;
        public double x2 = 48;
        public double y2 = -32;
        public double h2 = Math.toRadians(180);
    }
    public static Params params = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(params.x1, params.y1, params.h1);
        Vector2d pos1 = new Vector2d(params.x2, params.y2);

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED, beginPose);
        PinpointDrive drive = robot.getDriveTrain();

        Action driveStrafe = drive.actionBuilder(beginPose)
                .strafeToLinearHeading(pos1, params.h2)
                .build();

        waitForStart();

        Actions.runBlocking(new SequentialAction(
                driveStrafe
        ));
    }
}
