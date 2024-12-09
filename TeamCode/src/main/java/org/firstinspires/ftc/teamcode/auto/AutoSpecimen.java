package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

import java.util.Arrays;

@Autonomous
@Config
public class AutoSpecimen extends LinearOpMode {
    public static class Params {
        public double beginX = 7.5, beginY = -64.5, beginA = Math.toRadians(90);
        public double hang1X = 0, hang1Y = -21.8; // first hang is a strafe to

        public double leftBlockX1 = 17.5, leftBlockY1 = -39, leftBlockA1 = Math.toRadians(180);
        public double leftBlockX2 = 47, leftBlockY2 = -13, leftBlockA2 = Math.toRadians(90), leftBlockT2 = Math.toRadians(270);
        public double leftBlockPushDistance = 40;
        public double hangLeftX = 5, hangLeftY = -21.8, hangLeftA = Math.toRadians(90);

        public double midBlockX = 60, midBlockY = -15, midBlockA = Math.toRadians(90), midBlockT = Math.toRadians(90); // try to make it so the block can go in the divet in the back of the robot??:
        public double midBlockPushDistance = 40;
    }
    public static Params params = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(params.beginX, params.beginY, params.beginA);
        Pose2d hang1Pose = new Pose2d(params.hang1X, params.hang1Y, params.beginA);

        Pose2d leftBlockPose1 = new Pose2d(params.leftBlockX1, params.leftBlockY1, params.leftBlockA1);
        Pose2d leftBlockPose2 = new Pose2d(params.leftBlockX2, params.leftBlockY2, params.leftBlockA2);
        Pose2d leftBlockAfterPose = new Pose2d(params.leftBlockX2, params.leftBlockY2 - params.leftBlockPushDistance, params.leftBlockA2);
        Pose2d hangLeftPose = new Pose2d(params.hangLeftX, params.hangLeftY, params.hangLeftA);

        Pose2d midBlockPose = new Pose2d(params.midBlockX, params.midBlockY, params.midBlockA);
        Pose2d midBlockAfterPose = new Pose2d(params.midBlockX, params.midBlockY - params.midBlockPushDistance, params.midBlockA);

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED, beginPose);
        PinpointDrive drive = robot.getDriveTrain();

        // hang first specimen
        TrajectoryActionBuilder driveToSpecimen1HangTrajectory = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(params.hang1X, params.hang1Y));

        // push first block into human player area
        TrajectoryActionBuilder pushLeftBlockTrajectory = drive.actionBuilder(hang1Pose)
                .strafeTo(new Vector2d(params.leftBlockX1, params.leftBlockY1))
                .setReversed(true)
                .splineToLinearHeading(leftBlockPose2, params.leftBlockT2)
                .lineToY(leftBlockAfterPose.position.y);

        // push second block into human player area
        TrajectoryActionBuilder pushMidBlockTrajectory = drive.actionBuilder(leftBlockAfterPose)
                .splineToLinearHeading(midBlockPose, params.midBlockT)
                .lineToY(midBlockAfterPose.position.y);

        // drive to hang left block position
        TrajectoryActionBuilder driveToLeftSpecimenHangTrajectory = drive.actionBuilder(midBlockAfterPose)
                .splineToLinearHeading(hangLeftPose, params.hangLeftA);

        Action driveToSpecimen1Hang = driveToSpecimen1HangTrajectory.build();
        Action pushLeftBlock = pushLeftBlockTrajectory.build();
        Action pushMidBlock = pushMidBlockTrajectory.build();
        Action driveToLeftSpecimenHang = driveToLeftSpecimenHangTrajectory.build();


        // setup the robot
        Actions.runBlocking(
                robot.getLiftingSystem().setupHighSpecimenRam()
        );

        waitForStart();

        // actual auto path
        Actions.runBlocking(new SequentialAction(
                // hang first specimen
                driveToSpecimen1Hang,
                robot.getLiftingSystem().ramHighSpecimen(),

                // push first block into human player area
                new ParallelAction(
                        robot.getLiftingSystem().resetSpecimenRam(),
                        pushLeftBlock
                )

                /*
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
                )
                //robot.getLiftingSystem().ramHighSpecimen()
                 */
        ));

    }
}