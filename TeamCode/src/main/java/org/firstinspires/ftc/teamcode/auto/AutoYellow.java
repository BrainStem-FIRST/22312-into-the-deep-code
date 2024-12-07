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
import org.firstinspires.ftc.teamcode.robot.Lift;

@Autonomous(name="AutoYellow")
@Config
public class AutoYellow extends LinearOpMode {
    public static class Params {
        public double beginX = -40.5, beginY = -64.5, beginA = 0;
        public double depositX = -58, depositY = -58, depositA = Math.toRadians(45), depositT = Math.toRadians(225);
        public double rightBlockX = -48.5, rightBlockY = -40, rightBlockA = Math.toRadians(90), rightBlockT = Math.toRadians(90);
        public double midBlockX = -60.5, midBlockY = -40, midBlockA = Math.toRadians(90), midBlockT = Math.toRadians(90);
        public double leftBlockX2 = -40, leftBlockY2 = -32.5, leftBlockX1 = -54, leftBlockY1 = -37.5, leftBlockA = Math.toRadians(135), leftBlockT = Math.toRadians(135);
        public double parkX = -12, parkY = 0, parkA = Math.toRadians(90);

        public int rightBlockExtensionAmount = 400;
        public int midBlockExtensionAmount = 400;
        public int leftBlockExtensionAmount = 400;

        public double rightBlockDriveForwardDistance = 8;
        public double midBlockDriveForwardDistance = 8;
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
                .splineToLinearHeading(parkPose, Math.toRadians(0));

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
                        new ParallelAction(
                                robot.getGrabber().close(),
                                new SequentialAction(
                                        robot.getLift().moveTo(Lift.TROUGH_SAFETY_POS),
                                        robot.getArm().rotateTo(Arm.SPECIMEN_HANG_POS, Arm.SPECIMEN_HANG_TO_UP_TIME + Arm.UP_TO_TRANSFER_TIME)
                                )
                        )
                )
        );

        telemetry.addLine("Robot Ready");
        telemetry.update();

        waitForStart();

        Actions.runBlocking(
                new SequentialAction(
                        // DEPOSIT FIRST BLOCK
                        // depositing block that you start with
                        new ParallelAction(
                                robot.getExtension().retractAction(),
                                firstDeposit,
                                robot.getArm().rotateTo(Arm.BASKET_SAFETY_POS, Arm.SPECIMEN_HANG_TO_UP_TIME + Arm.UP_TO_BASKET_SAFETY_TIME),
                                robot.getLiftingSystem().depositHighInitial()
                        ),
                        // GET AND DEPOSIT RIGHT BLOCK
                        // resetting lift, driving to position and extending and collecting right block
                        new ParallelAction(
                                new SequentialAction(
                                        rightBlock,
                                        rightBlockDriveThrough
                                ),
                                robot.getCollectingSystem().extendAndCollectAction(params.rightBlockExtensionAmount),
                                robot.getLiftingSystem().lowerFromDeposit()
                        ),
                        // retracting extension, transferring block to lift, and going to deposit position and depositing
                        new ParallelAction(
                                robot.retractAndTransferAndDeposit(),
                                rightBlockDeposit
                        ),

                        // GET AND DEPOSIT MIDDLE BLOCK
                        new ParallelAction(
                                new SequentialAction(
                                        midBlock, midBlockDriveThrough
                                ),
                                robot.getLiftingSystem().lowerFromDeposit(),
                                robot.getCollectingSystem().extendAndCollectAction(params.midBlockExtensionAmount)
                        ),
                        new ParallelAction(
                                robot.retractAndTransferAndDeposit(),
                                midBlockDeposit
                        ),

                        // GET AND DEPOSIT LEFT BLOCK
                        new ParallelAction(
                                new SequentialAction(
                                        leftBlock,
                                        leftBlockDriveThrough
                                ),
                                robot.getLiftingSystem().lowerFromDeposit(),
                                robot.getCollectingSystem().extendAndCollectAction(params.leftBlockExtensionAmount)
                        ),
                        new ParallelAction(
                                robot.retractAndTransferAndDeposit(),
                                leftBlockDeposit
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