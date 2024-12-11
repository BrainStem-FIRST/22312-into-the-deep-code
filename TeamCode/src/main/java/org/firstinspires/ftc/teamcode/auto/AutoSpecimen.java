package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileParams;
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
public class AutoSpecimen extends LinearOpMode {
    public static class Params {
        public double dispResolution = 4, angResolution = Math.toRadians(20), anglSamplingEps = 1e-1, beginEndVel=0;

        public double beginX = 7.175, beginY = -64.5, beginA = Math.toRadians(90);
        public double hang1X = 0, hang1Y = -28; // first hang is a strafe to

        public double leftBlockX1 = 38, leftBlockY1 = -39, leftBlockT1 = Math.toRadians(90);
        public double leftBlockX2 = 47, leftBlockY2 = -15, leftBlockT2 = Math.toRadians(0);
        public double leftBlockPushDistance = 40;
        public double hangLeftX = 3, hangLeftY = hang1Y, hangLeftA = Math.toRadians(90);

        public double midBlockX = 57, midBlockY = leftBlockY2, midBlockA = Math.toRadians(90), midBlockT = Math.toRadians(0); // try to make it so the block can go in the divet in the back of the robot??:
        public double midBlockPushDistance = 40;
    }
    public static Params params = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(params.beginX, params.beginY, params.beginA);
        Pose2d hang1Pose = new Pose2d(params.hang1X, params.hang1Y, params.beginA);

        Pose2d leftBlockPose1 = new Pose2d(params.leftBlockX1, params.leftBlockY1, params.beginA);
        Pose2d leftBlockPose2 = new Pose2d(params.leftBlockX2, params.leftBlockY2, params.beginA);
        Pose2d leftBlockAfterPose = new Pose2d(params.leftBlockX2, params.leftBlockY2 - params.leftBlockPushDistance, params.beginA);
        Pose2d hangLeftPose = new Pose2d(params.hangLeftX, params.hangLeftY, params.hangLeftA);

        Pose2d midBlockPose = new Pose2d(params.midBlockX, params.midBlockY, params.midBlockA);
        Pose2d midBlockAfterPose = new Pose2d(params.midBlockX, params.midBlockY - params.midBlockPushDistance, params.midBlockA);

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED, beginPose);
        PinpointDrive drive = robot.getDriveTrain();

        ProfileParams wayPointParams = new ProfileParams(params.dispResolution, params.angResolution, params.anglSamplingEps);

        // hang first specimen
        TrajectoryActionBuilder driveToSpecimen1HangTrajectory = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(params.hang1X, params.hang1Y));

        // push first block into human player area
        TrajectoryActionBuilder pushLeftBlockTrajectory = drive.waypointActionBuilder(hang1Pose, wayPointParams, params.beginEndVel)
                .splineToLinearHeading(leftBlockPose1, params.leftBlockT1)
                .splineToLinearHeading(leftBlockPose2, params.leftBlockT2)
                .strafeTo(leftBlockAfterPose.position);

        // push second block into human player area
        TrajectoryActionBuilder pushMidBlockTrajectory = drive.actionBuilder(leftBlockAfterPose)
                .splineToLinearHeading(midBlockPose, params.midBlockT)
                .strafeTo(midBlockAfterPose.position);

        // drive to hang left block position
        TrajectoryActionBuilder driveToLeftSpecimenHangTrajectory = drive.actionBuilder(midBlockAfterPose)
                .splineToLinearHeading(hangLeftPose, params.hangLeftA);

        Action driveToSpecimen1Hang = driveToSpecimen1HangTrajectory.build();
        Action pushLeftBlock = pushLeftBlockTrajectory.build();
        Action pushMidBlock = pushMidBlockTrajectory.build();
        Action driveToLeftSpecimenHang = driveToLeftSpecimenHangTrajectory.build();

        waitForStart();

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

    }
}
