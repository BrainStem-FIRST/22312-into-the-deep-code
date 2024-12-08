package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous(name="AutoYellow")
@Config
public class AutoYellow extends LinearOpMode {
    public static class Params {
        public double beginX = -40.825, beginY = -64.5, beginA = 0;
        public double depositX = -58.5, depositY = -59.5, depositA = Math.toRadians(45), depositT = Math.toRadians(225);
        public double rightBlockX = -49.5, rightBlockY = -44, rightBlockA = Math.toRadians(90), rightBlockT = Math.toRadians(90);
        public double midBlockX = -61.8, midBlockY = -42, midBlockA = Math.toRadians(90), midBlockT = Math.toRadians(90);
        public double leftBlockX1 = -58.7, leftBlockY1 = -40.5, leftBlockX2 = 60.5, leftBlockY2 = -26, leftBlockA = Math.toRadians(135);
        public double parkX = -12, parkY = 0, parkA = Math.toRadians(0), parkT = Math.toRadians(0);
        public int rightBlockExtensionAmount = Extension.MIN_SEARCH_AND_COLLECT_POSITION;
        public int midBlockExtensionAmount = Extension.MIN_SEARCH_AND_COLLECT_POSITION;
        public int leftBlockExtensionAmount = Extension.MIN_SEARCH_AND_COLLECT_POSITION;

        public double rightBlockDriveForwardDistance = 7;
        public double midBlockDriveForwardDistance = 7;
    }
    public static Params params = new Params();
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Pose2d beginPose = new Pose2d(params.beginX, params.beginY, params.beginA);
        Pose2d depositPose = new Pose2d(params.depositX, params.depositY, params.depositA);
        Pose2d rightBlockPose = new Pose2d(params.rightBlockX, params.rightBlockY, params.rightBlockA);
        Pose2d rightBlockCollectedPose = new Pose2d(params.rightBlockX, params.rightBlockY + params.rightBlockDriveForwardDistance, params.rightBlockA);
        Pose2d midBlockPose = new Pose2d(params.midBlockX, params.midBlockY, params.midBlockA);
        Pose2d midBlockCollectedPose = new Pose2d(params.rightBlockX, params.rightBlockY + params.midBlockDriveForwardDistance, params.midBlockA);
        Pose2d leftBlockPose = new Pose2d(params.leftBlockX1, params.leftBlockY1, params.leftBlockA);
        Pose2d leftBlockCollectedPose = new Pose2d(params.leftBlockX2, params.leftBlockY2, params.leftBlockA);
        Pose2d parkPose = new Pose2d(params.parkX, params.parkY, params.parkA);

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED, beginPose);
        PinpointDrive drive = robot.getDriveTrain();

        // going to deposit position
        TrajectoryActionBuilder firstDepositTrajectory = drive.actionBuilder(beginPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, params.depositT);

        TrajectoryActionBuilder rightBlockDepositTrajectory = drive.actionBuilder(rightBlockCollectedPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, params.depositT);

        TrajectoryActionBuilder midBlockDepositTrajectory = drive.actionBuilder(midBlockCollectedPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, params.depositT);

        TrajectoryActionBuilder leftBlockDepositTrajectory = drive.actionBuilder(leftBlockCollectedPose)
                .setReversed(true)
                .splineToLinearHeading(depositPose, params.depositT);


        TrajectoryActionBuilder rightBlockTrajectory = drive.actionBuilder(depositPose)
                .splineToLinearHeading(rightBlockPose, params.rightBlockT);
        TrajectoryActionBuilder rightBlockDriveThroughTrajectory = drive.actionBuilder(rightBlockPose)
                .lineToY(params.rightBlockY + params.rightBlockDriveForwardDistance);

        TrajectoryActionBuilder midBlockTrajectory = drive.actionBuilder(depositPose)
                .splineToLinearHeading(midBlockPose, params.midBlockT);
        TrajectoryActionBuilder midBlockDriveThroughTrajectory = drive.actionBuilder(midBlockPose)
                .lineToY(params.midBlockY + params.midBlockDriveForwardDistance);

        TrajectoryActionBuilder leftBlockTrajectory = drive.actionBuilder(depositPose)
                .strafeToLinearHeading(new Vector2d(params.leftBlockX1, params.leftBlockY1), params.leftBlockA);
                //.splineToLinearHeading(leftBlockPose, params.leftBlockT);
        TrajectoryActionBuilder leftBlockDriveThroughTrajectory = drive.actionBuilder(leftBlockPose)
                .strafeToConstantHeading(new Vector2d(params.leftBlockX2, params.leftBlockY2));
                //.lineToX(params.leftBlockX - params.leftBlockDriveForwardDistance);

        TrajectoryActionBuilder parkTrajectory = drive.actionBuilder(depositPose)
                .splineToLinearHeading(parkPose, params.parkT);

        Action firstDeposit = firstDepositTrajectory.build();

        Action rightBlock = rightBlockTrajectory.build();
        Action rightBlockDeposit = rightBlockDepositTrajectory.build();
        Action rightBlockDriveThrough = rightBlockDriveThroughTrajectory.build();

        Action midBlock = midBlockTrajectory.build();
        Action midBlockDeposit = midBlockDepositTrajectory.build();
        Action midBlockDriveThrough = midBlockDriveThroughTrajectory.build();

        Action leftBlock = leftBlockTrajectory.build();
        Action leftBlockDeposit = leftBlockDepositTrajectory.build();
        Action leftBlockDriveThrough = leftBlockDriveThroughTrajectory.build();

        Action park = parkTrajectory.build();

        // setting up robot
        Actions.runBlocking(
                new SequentialAction(
                        robot.getGrabber().close(),
                        new SequentialAction(
                                // setting up lift and arm
                                robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS),
                                robot.getArm().rotateTo(Arm.SPECIMEN_HANG_POS, Arm.SPECIMEN_HANG_TO_UP_TIME + Arm.UP_TO_TRANSFER_TIME),

                                // setting up extension and hinge
                                robot.getHinge().hingeUpAction(),
                                robot.getExtension().retractAction()
                        )
                )
        );

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        // DEPOSIT FIRST BLOCK & PREP FOR RIGHT BLOCK COLLECTION
                        // depositing block that you start with
                        // extend and hinge down while doing this
                        new ParallelAction(
                                firstDeposit,
                                robot.getArm().rotateTo(Arm.BASKET_SAFETY_POS, Arm.SPECIMEN_HANG_TO_UP_TIME + Arm.UP_TO_BASKET_SAFETY_TIME),
                                robot.getLiftingSystem().depositHighInitial(),
                                robot.getCollectingSystem().startCollectSequence(params.rightBlockExtensionAmount)
                        ),
                        // GET AND DEPOSIT RIGHT BLOCK
                        // resetting lift, driving to position and extending and collecting right block
                        new ParallelAction(
                                robot.getLiftingSystem().lowerFromDeposit(),

                                robot.getCollectingSystem().startCollect(),
                                new SequentialAction(
                                        rightBlock,
                                        rightBlockDriveThrough
                                )
                        ),
                        // retracting extension, transferring block to lift, going to deposit position and depositing, and doing a short extend and hinge
                        new ParallelAction(
                                rightBlockDeposit,
                                robot.retractAndDepositAndExtend(params.midBlockExtensionAmount)
                        ),

                        // GET AND DEPOSIT MIDDLE BLOCK
                        new ParallelAction(
                                robot.getLiftingSystem().lowerFromDeposit(),
                                robot.getCollectingSystem().startCollect(),
                                new SequentialAction(
                                        midBlock,
                                        midBlockDriveThrough
                                )
                        ),
                        new ParallelAction(
                                midBlockDeposit,
                                robot.retractAndDepositAndExtend(params.leftBlockExtensionAmount)
                        ),

                        // GET AND DEPOSIT LEFT BLOCK
                        new ParallelAction(
                                robot.getLiftingSystem().lowerFromDeposit(),
                                robot.getCollectingSystem().startCollect(),
                                new SequentialAction(
                                        leftBlock,
                                        leftBlockDriveThrough
                                )
                        ),
                        new ParallelAction(
                                leftBlockDeposit,
                                robot.retractAndDeposit()
                        ),

                        // PARK ROBOT
                        new ParallelAction(
                                robot.getLiftingSystem().lowerFromDeposit(),
                                park
                        )
                )
        );
    }
}