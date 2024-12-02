package org.firstinspires.ftc.teamcode.driveTrain.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.driveTrain.useless.TankDrive;

@Config
public final class SplineTest extends LinearOpMode {
    public static class Params {
        public double beginX = 31.5, beginY = -64.5, beginA = Math.PI/2;
        public double hangX = 0, hangY = -21.8, hangA = Math.PI/2;
    }

    public static Params PARAMS = new Params();
    @Override
    public void runOpMode() throws InterruptedException {
        /*
        Pose2d beginPose = new Pose2d(PARAMS.beginX, PARAMS.beginY, PARAMS.beginA);
        if (TuningOpModes.DRIVE_CLASS.equals(PinpointDrive.class)) {
            PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(PARAMS.hangX, PARAMS.hangY), PARAMS.hangA)
                            .build());*/

        Pose2d beginPose = new Pose2d(PARAMS.beginX, PARAMS.beginY, PARAMS.beginA);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(PARAMS.hangX, PARAMS.hangY), PARAMS.hangA)
                        .build());
    }
}