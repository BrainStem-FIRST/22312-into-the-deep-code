package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

@Autonomous
@Config
public class AutoSpecimenSplines extends LinearOpMode {
    public static class Params {

        public double wallPickupX = 0, wallPickupY = 0, wallPickupA = Math.toRadians(90), wallPickupT = Math.toRadians(90);
        public double hang1X = -36, hang1Y = 40, hang1A = Math.toRadians(90);
    }
    public static Params params = new Params();

    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d wallPickupPose = new Pose2d(params.wallPickupX, params.wallPickupY, params.wallPickupA);
        Pose2d hang1Pose = new Pose2d(params.hang1X, params.hang1Y, params.hang1A);

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED, wallPickupPose);
        PinpointDrive drive = robot.getDriveTrain();
        drive.setParamsForSpecimen();


        TrajectoryActionBuilder wallToHangTrajectory = drive.actionBuilder(wallPickupPose)
                .strafeTo(new Vector2d(params.hang1X, params.hang1Y));
        TrajectoryActionBuilder hangToWallTrajectory = drive.actionBuilder(hang1Pose)
                .splineToConstantHeading(new Vector2d(params.wallPickupX, params.wallPickupY), params.wallPickupT);

        Action wallToHang = wallToHangTrajectory.build();
        Action hangToWall = hangToWallTrajectory.build();
        Action wallToHang2 = wallToHangTrajectory.build();
        Action hangToWall2 = hangToWallTrajectory.build();

        waitForStart();

        Actions.runBlocking(new ParallelAction(
                new SequentialAction(
                        wallToHang,
                        new SleepAction(0.5),
                        hangToWall,
                        new SleepAction(0.5),
                        wallToHang2,
                        new SleepAction(0.5),
                        hangToWall2
                ),
                new Action() {
                    int counter = 0;
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                        counter++;
                        telemetry.addData("counter", counter);
                        telemetry.update();
                        return counter < 5000;
                    }
                }
        ));
        /*
        // actual auto path
        Actions.runBlocking(new SequentialAction(
                // hang first specimen
                driveToSpecimen1Hang,
                //robot.getLiftingSystem().ramHighSpecimen(),
                new SleepAction(0.5),
                // push first block into human player area
                new ParallelAction(
                        //robot.getLiftingSystem().resetSpecimenRam(),
                        pushLeftBlock
                ),


                // push second block into human player
                new ParallelAction(
                        pushMidBlock
                        //robot.getLiftingSystem().transferToDropOff()
                ),


                //get first specimen from wall and hang it on bar
                //robot.getGrabber().close(),
                new ParallelAction(
                        driveToLeftSpecimenHang
                        //robot.getLiftingSystem().setupHighSpecimenRam()
                ),

                new SleepAction(0.5)
                //robot.getLiftingSystem().ramHighSpecimen()
        ));
         */

    }
}
