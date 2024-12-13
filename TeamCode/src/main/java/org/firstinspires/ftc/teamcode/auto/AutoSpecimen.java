package org.firstinspires.ftc.teamcode.auto;

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
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;

@Autonomous
@Config
public class AutoSpecimen extends LinearOpMode {
    public static class DriveParams {
        public double dispResolution = 4, angResolution = Math.toRadians(20), anglSamplingEps = 1e-1, beginEndVel=0;
        public double translationalVelConstraint = 70;
    }
    public static class Params {

        public double beginX = 7.175, beginY = -64.5, beginA = Math.toRadians(90);
        public double hang0X = -5, hang0Y = -28,
                hang1X = -8, hang1Y = -28,
                hang2X = -7, hang2Y = -28,
                hang3X = -6, hang3Y = -28,
                hangA = Math.toRadians(90), hangT = Math.toRadians(90);
        public double hangAlignDrivePower = 0.15;
        public double leftBlockX1 = 38, leftBlockY1 = -39, leftBlockT1 = Math.toRadians(90), leftBlockMinVel1 = driveParams.translationalVelConstraint;
        public double leftBlockX2 = 47, leftBlockY2 = -15, leftBlockT2 = Math.toRadians(0), leftBlockMinVel2 = driveParams.translationalVelConstraint;
        public double leftBlockPushDistance = 40, leftBlockT3 = Math.toRadians(90), leftBlockMinVel3 = driveParams.translationalVelConstraint;

        public double midBlockX = 57, midBlockY = leftBlockY2, midBlockA = Math.toRadians(90), midBlockT = Math.toRadians(0), midBlockMinVel1 = driveParams.translationalVelConstraint; // try to make it so the block can go in the divet in the back of the robot??:
        public double midBlockPushDistance = 40, midBlockMinVel2 = driveParams.translationalVelConstraint;

        public double wallPickupWaitTime = 0.3;
        public double wallPickupX1 = 36, wallPickupY1 = -70,
                wallPickupX2 = 39, wallPickupY2 = -72,
                wallPickupX3 = 42, wallPickupY3 = -74,
                wallPickupA = Math.toRadians(90), wallToHangMinVel = driveParams.translationalVelConstraint, hangToWallMinVel = driveParams.translationalVelConstraint;
    }
    public static DriveParams driveParams = new DriveParams();
    public static Params params = new Params();

    private Action createSpecimenHangCycle(BrainSTEMRobot robot, Action driveToHang, Action driveToPickup) {
        return new SequentialAction(
                // pickup and hang specimen at submersible
                new SleepAction(params.wallPickupWaitTime),
                robot.getGrabber().close(),
                new ParallelAction(
                        driveToHang,
                        robot.getLiftingSystem().setupHighSpecimenRam()
                ),
                new ParallelAction(
                        robot.getLiftingSystem().ramHighSpecimen(),
                        keepRobotAligned(robot)
                ),

                // reset lifting system and go back to wall
                robot.getGrabber().open(),
                new ParallelAction(
                        driveToPickup,
                        new SequentialAction(
                                robot.getLiftingSystem().resetSpecimenRam(),
                                robot.getLiftingSystem().transferToDropOff()
                        )
                )
        );
    }

    // keep robot aligned while hanging specimens
    private Action keepRobotAligned(BrainSTEMRobot robot) {
        return telemetryPacket -> {
            robot.getDriveTrain().setMotorPowers(params.hangAlignDrivePower, params.hangAlignDrivePower, params.hangAlignDrivePower, params.hangAlignDrivePower);
            return !Subsystem.inRange(robot.getLift().getLiftMotor(), Lift.HIGH_RAM_AFTER_POS,Lift.AUTO_DESTINATION_THRESHOLD);
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
        Pose2d leftBlockAfterPose = new Pose2d(params.leftBlockX2, params.leftBlockY2 - params.leftBlockPushDistance, params.beginA);

        Pose2d midBlockPose = new Pose2d(params.midBlockX, params.midBlockY, params.midBlockA);
        Pose2d midBlockAfterPose = new Pose2d(params.midBlockX, params.midBlockY - params.midBlockPushDistance, params.midBlockA);

        Pose2d wallPickupPose1 = new Pose2d(params.wallPickupX1, params.wallPickupY1, params.wallPickupA);
        Pose2d wallPickupPose2 = new Pose2d(params.wallPickupX2, params.wallPickupY2, params.wallPickupA);
        Pose2d wallPickupPose3 = new Pose2d(params.wallPickupX3, params.wallPickupY3, params.wallPickupA);

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED, beginPose);
        PinpointDrive drive = robot.getDriveTrain();

        ProfileParams wayPointParams = new ProfileParams(driveParams.dispResolution, driveParams.angResolution, driveParams.anglSamplingEps);
        TranslationalVelConstraint leftBlock1VelConstraint = new TranslationalVelConstraint(params.leftBlockMinVel1);
        TranslationalVelConstraint leftBlock2VelConstraint = new TranslationalVelConstraint(params.leftBlockMinVel2);
        TranslationalVelConstraint leftBlock3VelConstraint = new TranslationalVelConstraint(params.leftBlockMinVel3);
        TranslationalVelConstraint midBlock1VelConstraint = new TranslationalVelConstraint(params.midBlockMinVel1);
        TranslationalVelConstraint midBlock2VelConstraint = new TranslationalVelConstraint(params.midBlockMinVel2);
        TranslationalVelConstraint wallToHangVelConstraint = new TranslationalVelConstraint(params.wallToHangMinVel);
        TranslationalVelConstraint hangToWallVelConstraint = new TranslationalVelConstraint(params.hangToWallMinVel);



        // hang first specimen
        TrajectoryActionBuilder driveToSpecimenHang0Trajectory = drive.actionBuilder(beginPose)
                .strafeTo(new Vector2d(params.hang0X, params.hang0Y));

        // push first block into human player area
        TrajectoryActionBuilder pushLeftBlockTrajectory = drive.waypointActionBuilder(hang0Pose, wayPointParams, driveParams.beginEndVel)
                .splineToLinearHeading(leftBlockPose1, params.leftBlockT1, leftBlock1VelConstraint)
                .splineToLinearHeading(leftBlockPose2, params.leftBlockT2, leftBlock2VelConstraint)
                .splineToLinearHeading(leftBlockAfterPose, params.leftBlockT3, leftBlock3VelConstraint);

        // push second block into human player area
        TrajectoryActionBuilder pushMidBlockTrajectory = drive.actionBuilder(leftBlockAfterPose)
                .splineToLinearHeading(midBlockPose, params.midBlockT, midBlock1VelConstraint)
                .strafeTo(midBlockAfterPose.position, midBlock2VelConstraint);

        // drive from wall pickup positions to specimen hang positions
        TrajectoryActionBuilder driveToSpecimenHang1Trajectory = drive.actionBuilder(midBlockAfterPose)
                .splineToLinearHeading(hang1Pose, params.hangT, wallToHangVelConstraint);
        TrajectoryActionBuilder driveToSpecimenHang2Trajectory = drive.actionBuilder(wallPickupPose1)
                .splineToLinearHeading(hang2Pose, params.hangT, wallToHangVelConstraint);
        TrajectoryActionBuilder driveToSpecimenHang3Trajectory = drive.actionBuilder(wallPickupPose2)
                .splineToLinearHeading(hang3Pose, params.hangT, wallToHangVelConstraint);

        // drive from specimen hang to wall pickup positions
        TrajectoryActionBuilder driveToPickup1Trajectory = drive.actionBuilder(hang1Pose)
                .strafeToConstantHeading(new Vector2d(params.wallPickupX1, params.wallPickupY1), hangToWallVelConstraint);
        TrajectoryActionBuilder driveToPickup2Trajectory = drive.actionBuilder(hang2Pose)
                .strafeToConstantHeading(new Vector2d(params.wallPickupX2, params.wallPickupY2), hangToWallVelConstraint);
        TrajectoryActionBuilder driveToPickup3Trajectory = drive.actionBuilder(hang3Pose)
                .strafeToConstantHeading(new Vector2d(params.wallPickupX3, params.wallPickupY3), hangToWallVelConstraint);

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
        Actions.runBlocking(new SequentialAction(
                // hang first specimen
                driveToSpecimen1Hang,
                new ParallelAction(
                        robot.getLiftingSystem().ramHighSpecimen(),
                        keepRobotAligned(robot)
                ),
                // push first block into human player area
                new ParallelAction(
                        robot.getLiftingSystem().resetSpecimenRam(),
                        pushLeftBlock
                ),

                // push second block into human player
                new ParallelAction(
                        pushMidBlock,
                        robot.getLiftingSystem().transferToDropOff()
                ),

                // repeat 4x
                //get specimen from wall and hang it on bar
                // go back to wall to pick up another specimen
                cycleSpecimenHang1,
                cycleSpecimenHang2,
                cycleSpecimenHang3
        ));

    }
}
