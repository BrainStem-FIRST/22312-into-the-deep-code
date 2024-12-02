package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
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
public class TestAuto extends LinearOpMode {
    public static class Params {
        public double beginX = -1, beginY = -64.5, beginA = Math.PI/2;
        public double hangX1 = -1, hangY1 = -29.5, hangA1 = -Math.PI/2;
        public double hangX2 = 0, hangY2 = -29.5, hangA2 = Math.PI/2;
        public double hangX3 = 3, hangY3 = -29.5, hangA3 = Math.PI/2;
        public double blockOneX = 48, blockOneY = -48, blockOneA = -Math.PI/2; // this (and block positions below) describe where to center robot (nothing about extension)
        public double blockTwoX = 60, blockTwoY = -48, blockTwoA = Math.PI/2;
        public double blockThreeX1 = 36, blockThreeY1 = -24, blockThreeA1 = Math.PI/2; // splineToConstantHeading
        public double blockThreeX2 = 64.5, blockThreeY2 = -15, blockThreeA2 = Math.PI*1.5; // splineToConstantHeading
        public double dropOffX = 64.5, dropOffY = -60, dropOffA = Math.PI/2; // place to drop off block in human player area
        public double pickUpX1 = 48, pickUpY1 = -60, pickUpA1 = Math.PI/2; // place to move robot so human player can adjust specimen
        public double pickUpX2 = 48, pickUpY2 = -64.5, pickUpA2 = Math.PI/2; // place for grabber to collect specimen
        public int extendMotorEncoder = 1600; // encoder value to move extension to

    }
    public static Params PARAMS = new Params();
    BrainSTEMRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(PARAMS.beginX, PARAMS.beginY, PARAMS.beginA);

        robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED, beginPose);
        PinpointDrive drive = robot.getDriveTrain();


        Action hangSpecimen = drive.actionBuilder(beginPose)
                .lineToY(PARAMS.hangY1)
                .stopAndAdd(robot.getLiftingSystem().ramHighSpecimen())
                .build();

        Action goToFirstBlock = new ParallelAction(
                robot.getLiftingSystem().resetSpecimenRam(),
                drive.actionBuilder(new Pose2d(PARAMS.hangX1, PARAMS.hangY1, PARAMS.hangA1))
                        .setReversed(true)
                        .splineTo(new Vector2d(PARAMS.blockOneX, PARAMS.blockOneY), PARAMS.blockOneA)
                        .build()
        );

        Action driveAndCollectFirstBlock = new ParallelAction(
                robot.getCollectingSystem().extendAndCollectAction(PARAMS.extendMotorEncoder)

        );


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                hangSpecimen,
                goToFirstBlock
        ));

        /*
        Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        // driving to specimen ramming position
                        .stopAndAdd(drive.lineToPos(new Vector2d(PARAMS.hangX1, PARAMS.hangY1), PARAMS.hangA1))

                        // ramming specimen you start with
                        .stopAndAdd(robot.getLiftingSystem().ramHighSpecimen())

                        // resetting lift and driving to pickup 1st block
                        .stopAndAdd(new ParallelAction(
                                robot.getLiftingSystem().resetSpecimenRam(),
                                drive.lineToPos(new Vector2d(PARAMS.blockOneX, PARAMS.blockOneY), PARAMS.blockOneA)))

                        // picking up first block
                        .stopAndAdd(robot.getCollectingSystem().extendAndCollectAction(PARAMS.extendMotorEncoder))

                        // resetting extension, transferring, and driving to drop first block off
                        .stopAndAdd(new ParallelAction(
                                new SequentialAction(
                                    robot.getCollectingSystem().retractAction(),
                                    robot.getLiftingSystem().transferBlock(),
                                    robot.getLiftingSystem().transferToDropOff()),
                                drive.lineToPos(new Vector2d(PARAMS.dropOffX, PARAMS.dropOffY), PARAMS.dropOffA)))

                        // dropping first block off
                        .stopAndAdd(robot.getGrabber().open())

                        // driving and extending to pick up second block
                        .stopAndAdd(new ParallelAction(
                                robot.getCollectingSystem().extendAndCollectAction(PARAMS.extendMotorEncoder),
                                drive.lineToPos(new Vector2d(PARAMS.blockTwoX, PARAMS.blockTwoY), PARAMS.blockTwoA)))

                        // retracting, transferring, and driving to drop off second block
                        .stopAndAdd(new ParallelAction(
                                new SequentialAction(
                                    robot.getCollectingSystem().retractAction(),
                                    robot.getLiftingSystem().transferBlock(),
                                    robot.getLiftingSystem().transferToDropOff()),
                                drive.lineToPos(new Vector2d(PARAMS.dropOffX, PARAMS.dropOffY), PARAMS.dropOffA)))

                        // dropping second block off
                        .stopAndAdd(robot.getGrabber().open())

                        // driving to specimen pickup area to pick up second specimen (created by first block)
                        .splineToConstantHeading(new Vector2d(PARAMS.pickUpX1, PARAMS.pickUpY1), PARAMS.pickUpA1)
                        .lineToY(PARAMS.pickUpY2)

                        // picking up specimen and driving to ram it
                        .stopAndAdd(new SequentialAction(
                                robot.getGrabber().close(),
                                new ParallelAction(
                                    robot.getLiftingSystem().setupHighSpecimenRam(),
                                    drive.lineToPos(new Vector2d(PARAMS.hangX2, PARAMS.hangY2), PARAMS.hangA2))))

                        // ramming second specimen (but created from first block)
                        .stopAndAdd(robot.getLiftingSystem().ramHighSpecimen())

                        // resetting lift and setting up to push third block to human player station
                        .stopAndAdd(new ParallelAction(
                                new SequentialAction(
                                    robot.getLiftingSystem().resetSpecimenRam(),
                                    robot.getLiftingSystem().transferToDropOff()),
                                new SequentialAction(
                                    drive.splineToConstantHeading(new Vector2d(PARAMS.blockThreeX1, PARAMS.blockThreeY1), PARAMS.blockThreeA1),
                                    drive.splineToConstantHeading(new Vector2d(PARAMS.blockThreeX2, PARAMS.blockThreeY2), PARAMS.blockThreeA2))))

                        // pushing third block to drop off area
                        .lineToY(PARAMS.dropOffY)

                        // driving to specimen pickup area to pickup 3rd specimen (one created with second block) off wall
                        .splineTo(new Vector2d(PARAMS.pickUpX1, PARAMS.pickUpY1), PARAMS.pickUpA1)
                        .lineToY(PARAMS.pickUpY2)

                        // grabbing specimen
                        .stopAndAdd(robot.getGrabber().close())


                        .build());*/
    }
}
