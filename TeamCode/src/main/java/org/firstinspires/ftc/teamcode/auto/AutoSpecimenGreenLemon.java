package org.firstinspires.ftc.teamcode.auto;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

import java.util.Arrays;

@Autonomous
@Config
public class AutoSpecimenGreenLemon extends LinearOpMode {
    public static class DriveParams {
        public double dispResolution = 4, angResolution = Math.toRadians(20), anglSamplingEps = 1e-1;
    }
    public static class Params {

        public double waitTime = 3;
        public double beginX = 16.725, beginY = -64.5, beginA = Math.toRadians(90);
        public double hang0X = 2, hang0Y = -28, hang0VelConstraint = 38,
                hang1X = -3.5, hang1Y = -27,
                hang2X = -7, hang2Y = -27,
                hang3X = -11, hang3Y = -27,
                hangA = Math.toRadians(90), hangT = Math.toRadians(90), hangWaitSeconds = 0.1;
        public double hangAlignDrivePower = 0.6;
        public double leftBlockX1 = 38, leftBlockY1 = -41, leftBlockT1 = Math.toRadians(90), leftBlockMinVel1 = 60;
        public double leftBlockX2 = 43.5, leftBlockY2 = -15, leftBlockT2 = Math.toRadians(0), leftBlockMinVel2 = 70;
        public double leftBlockX3 = 43.5, leftBlockY3 = -50, leftBlockAAfter = Math.toRadians(80), leftBlockT3 = Math.toRadians(90), leftBlockMinVel3 = 65;

        public double midBlockX = 51, midBlockY = -15, midBlockA = Math.toRadians(90), midBlockT = Math.toRadians(0), midBlockMinVel1 = 65; // try to make it so the block can go in the divet in the back of the robot??:
        public double midBlockX2 = 51, midBlockY2 = -67, midBlockT2 = Math.toRadians(270), midBlockMinVel2 = 35;

        public double wallPickupWaitTime = 0.1, wallPickupGrabberWaitTime = 0.12;
        public double wallPickupX1 = 68, wallPickupY1 = -66,
                wallPickupX2 = 40.5, wallPickupY2 = -68.5,
                wallPickupX3 = 40.75, wallPickupY3 = -69.5,
                wallPickupA = Math.toRadians(90), wallPickupT = Math.toRadians(270),
                wallToHangMinVel = 50, hangToWallMinVel = 50, wallAlignPower = -0.85;
        public double delayTime = 5;
    }
    public static DriveParams driveParams = new DriveParams();
    public static Params params = new Params();

    private Action createSpecimenHangCycle(BrainSTEMRobot robot, Action driveToHang, Action driveToPickup) {
        return new SequentialAction(

                // pickup block from wall
                new ParallelAction(
                        keepRobotAlignedWall(robot),
                        new SequentialAction(
                                new SleepAction(params.wallPickupGrabberWaitTime),
                                robot.getGrabber().close()
                        )
                ),

                // hang block
                new ParallelAction(
                        driveToHang,
                        robot.getLiftingSystem().setupHighSpecimenRam()
                ),
                new ParallelAction(
                        keepRobotAlignedHang(robot),
                        new SequentialAction(
                                new SleepAction(params.hangWaitSeconds),
                                robot.getLiftingSystem().ramHighSpecimen()
                        )
                ),

                // reset lifting system and go back to wall
                robot.getGrabber().open(),
                new ParallelAction(
                        driveToPickup,
                        robot.getLiftingSystem().resetSpecimenRam()
                )
        );
    }

    // keep robot aligned while hanging specimens
    private Action keepRobotAlignedHang(BrainSTEMRobot robot) {
        return telemetryPacket -> {
            Log.d("DRIVE POWERS DURING HANG ALIGN", Arrays.toString(robot.getDriveTrain().getMotorPowers()));
            robot.getDriveTrain().setMotorPowers(params.hangAlignDrivePower, params.hangAlignDrivePower, params.hangAlignDrivePower, params.hangAlignDrivePower);
            return !Subsystem.inRange(robot.getLift().getLiftMotor(), Lift.HIGH_RAM_AFTER_POS,Lift.AUTO_DESTINATION_THRESHOLD);
        };
    }
    // keep robot aligned while picking up from wall
    private Action keepRobotAlignedWall(BrainSTEMRobot robot) {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();
            private boolean first = true;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (first) {
                    first = false;
                    timer.reset();
                }
                robot.getDriveTrain().setMotorPowers(params.wallAlignPower, params.wallAlignPower, params.wallAlignPower, params.wallAlignPower);
                return timer.seconds() < params.wallPickupWaitTime;
            }
        };
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(params.beginX, params.beginY, params.beginA);
        Pose2d hang0Pose = new Pose2d(params.hang0X, params.hang0Y, params.hangA);
        Pose2d hang1Pose = new Pose2d(params.hang1X, params.hang1Y, params.hangA);
        Pose2d hang2Pose = new Pose2d(params.hang2X, params.hang2Y, params.hangA);
        Pose2d hang3Pose = new Pose2d(params.hang3X, params.hang3Y, params.hangA);

        Pose2d leftBlockPose1 = new Pose2d(params.leftBlockX1, params.leftBlockY1, params.beginA);
        Pose2d leftBlockPose2 = new Pose2d(params.leftBlockX2, params.leftBlockY2, params.beginA);
        Pose2d leftBlockAfterPose = new Pose2d(params.leftBlockX3, params.leftBlockY3, params.leftBlockAAfter);

        Pose2d midBlockPose = new Pose2d(params.midBlockX, params.midBlockY, params.midBlockA);
        Pose2d midBlockAfterPose = new Pose2d(params.midBlockX2, params.midBlockY2, params.midBlockA);

        Pose2d wallPickupPose1 = new Pose2d(params.wallPickupX1, params.wallPickupY1, params.wallPickupA);
        Pose2d wallPickupPose2 = new Pose2d(params.wallPickupX2, params.wallPickupY2, params.wallPickupA);
        Pose2d wallPickupPose3 = new Pose2d(params.wallPickupX3, params.wallPickupY3, params.wallPickupA);

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED, beginPose);
        robot.getLift().getLiftMotor().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        PinpointDrive drive = robot.getDriveTrain();

        ProfileParams wayPointParams = new ProfileParams(driveParams.dispResolution, driveParams.angResolution, driveParams.anglSamplingEps);
        TranslationalVelConstraint hang0VelConstraint = new TranslationalVelConstraint(params.hang0VelConstraint);
        TranslationalVelConstraint leftBlock1VelConstraint = new TranslationalVelConstraint(params.leftBlockMinVel1);
        TranslationalVelConstraint leftBlock2VelConstraint = new TranslationalVelConstraint(params.leftBlockMinVel2);
        TranslationalVelConstraint leftBlock3VelConstraint = new TranslationalVelConstraint(params.leftBlockMinVel3);
        TranslationalVelConstraint midBlock1VelConstraint = new TranslationalVelConstraint(params.midBlockMinVel1);
        TranslationalVelConstraint midBlock2VelConstraint = new TranslationalVelConstraint(params.midBlockMinVel2);
        TranslationalVelConstraint wallToHangVelConstraint = new TranslationalVelConstraint(params.wallToHangMinVel);
        TranslationalVelConstraint hangToWallVelConstraint = new TranslationalVelConstraint(params.hangToWallMinVel);

        // hang first specimen
        TrajectoryActionBuilder driveToSpecimenHang0Trajectory = drive.actionBuilder(beginPose)
                .splineToConstantHeading(hang0Pose.position, params.beginA, hang0VelConstraint);

        // push first block into human player area
        TrajectoryActionBuilder pushLeftBlockTrajectory = drive.actionBuilder(hang0Pose)
                .splineToLinearHeading(leftBlockPose1, params.leftBlockT1, leftBlock1VelConstraint)
                //.splineToConstantHeading(leftBlockPose1.position, params.leftBlockT1)
                //.splineToConstantHeading(leftBlockPose2.position, params.leftBlockT2)
                .splineToLinearHeading(leftBlockPose2, params.leftBlockT2, leftBlock2VelConstraint)
                //.splineToLinearHeading(leftBlockAfterPose, params.leftBlockT3);
                .splineToLinearHeading(leftBlockAfterPose, params.leftBlockT3, leftBlock3VelConstraint);


        // push second block into human player area
        TrajectoryActionBuilder pushMidBlockTrajectory = drive.actionBuilder(leftBlockAfterPose)
                .splineToLinearHeading(midBlockPose, params.midBlockT, midBlock1VelConstraint)
                .splineToLinearHeading(midBlockAfterPose, params.midBlockT2, midBlock2VelConstraint);

        // drive from wall pickup positions to specimen hang positions
        TrajectoryActionBuilder driveToSpecimenHang1Trajectory = drive.actionBuilder(midBlockAfterPose)
                .splineToConstantHeading(hang1Pose.position, params.hangT, wallToHangVelConstraint);
        TrajectoryActionBuilder driveToSpecimenHang2Trajectory = drive.actionBuilder(wallPickupPose1)
                .splineToConstantHeading(hang2Pose.position, params.hangT, wallToHangVelConstraint);
        TrajectoryActionBuilder driveToSpecimenHang3Trajectory = drive.actionBuilder(wallPickupPose2)
                .splineToConstantHeading(hang3Pose.position, params.hangT, wallToHangVelConstraint);

        // drive from specimen hang to wall pickup positions
        TrajectoryActionBuilder driveToPickup1Trajectory = drive.actionBuilder(hang1Pose)
                .splineToConstantHeading(wallPickupPose1.position, params.wallPickupT, hangToWallVelConstraint);
        TrajectoryActionBuilder driveToPickup2Trajectory = drive.actionBuilder(hang2Pose)
                .splineToConstantHeading(wallPickupPose2.position, params.wallPickupT, hangToWallVelConstraint);
        TrajectoryActionBuilder driveToPickup3Trajectory = drive.actionBuilder(hang3Pose)
                .splineToConstantHeading(wallPickupPose3.position, params.wallPickupT, hangToWallVelConstraint);

        Action driveToSpecimen1Hang = driveToSpecimenHang0Trajectory.build();
        Action pushLeftBlock = pushLeftBlockTrajectory.build();
        Action pushMidBlock = pushMidBlockTrajectory.build();
        Action driveToSpecimenHang1 = driveToSpecimenHang1Trajectory.build();
        Action driveToSpecimenHang2 = driveToSpecimenHang2Trajectory.build();
        Action driveToSpecimenHang3 = driveToSpecimenHang3Trajectory.build();
        Action driveToPickup1 = driveToPickup1Trajectory.build();
        Action driveToPickup2 = driveToPickup2Trajectory.build();
        Action driveToPickup3 = driveToPickup3Trajectory.build();

        Action cycleSpecimenHang1 = createSpecimenHangCycle(robot, driveToSpecimenHang1, driveToPickup1);
        Action cycleSpecimenHang2 = createSpecimenHangCycle(robot, driveToSpecimenHang2, driveToPickup2);
        Action cycleSpecimenHang3 = createSpecimenHangCycle(robot, driveToSpecimenHang3, driveToPickup3);


        waitForStart();

        // actual auto path
        Actions.runBlocking(
                new SequentialAction(
                        new SleepAction(params.waitTime),
                    new ParallelAction(
                            robot.getExtension().retractContinuously(),
                            new SequentialAction(
                                    // hang first specimen
                                    new ParallelAction(
                                            driveToSpecimen1Hang,
                                            robot.getLiftingSystem().setupHighSpecimenRam()
                                    ),
                                    new ParallelAction(
                                            keepRobotAlignedHang(robot),
                                            new SequentialAction(
                                                    new SleepAction(params.hangWaitSeconds),
                                                    robot.getLiftingSystem().ramHighSpecimen()
                                            )
                                    ),
                                    // push first block into human player area
                                    new ParallelAction(
                                            robot.getLiftingSystem().resetSpecimenRam(),
                                            pushLeftBlock
                                    ),

                                    // push second block into human player
                                    pushMidBlock,

                                    // repeat 4x
                                    //get specimen from wall and hang it on bar
                                    // go back to wall to pick up another specimen
                                    cycleSpecimenHang1,
                                    cycleSpecimenHang2
                            )
                    )
                )
        );

    }
}
