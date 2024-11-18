package org.firstinspires.ftc.teamcode.auto;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;

@Autonomous
public abstract class AutoYellow extends Auto {

    // TODO: FIND EXTEND TICK POSITION
    public static int EXTEND_POSITION = 200;

    private final YellowBlockParams yellowBlockParams;
    public AutoYellow(YellowBlockParams yellowBlockParams) {
        super(AllianceColor.BLUE);

        this.yellowBlockParams = yellowBlockParams;
    }

    @Override
    public void runAuto() {
        // UNSURE
        robot.getDriveTrain().pose = yellowBlockParams.getBeginPose();

        runBlocking(
                new SequentialAction(
                        getBlockLineTrajectory(yellowBlockParams.getBlockOnePos(), yellowBlockParams.getBlockOneHeading())//,
                        //getBlockLineTrajectory(yellowBlockParams.getBlockTwoPos(), yellowBlockParams.getBlockTwoHeading()),
                        //getBlockSplineTrajectory(yellowBlockParams.getBlockThreePose(), yellowBlockParams.getBlockThreeTangent())
                )
        );
    }

    // moves in straight line and adjusts heading
    public Action getBlockLineTrajectory(Vector2d blockPosition, double heading) {
        return robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)
                // TODO: allow retracting lift and moving to happen concurrently
                .stopAndAdd(robot.getLiftingSystem().depositHigh())

                .stopAndAdd(lineToPos(blockPosition, heading))

                .stopAndAdd(robot.getCollectingSystem().extendAndCollectAction(EXTEND_POSITION))

                // retract and go to deposit position at same time
                .stopAndAdd(new ParallelAction(
                        robot.getExtension().retractAction(),
                        lineToPos(Auto.getPositionVector(yellowBlockParams.getBeginPose()), Auto.getHeading(yellowBlockParams.getBeginPose()))
                ))
                .build();
    }
    // moves in spline and adjusts heading
    public Action getBlockSplineTrajectory(Pose2d blockPose, double tangent) {
        return robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)
                // TODO: allow retracting lift and moving to happen concurrently
                .stopAndAdd(robot.getLiftingSystem().depositHigh())

                .stopAndAdd(robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)
                        .splineToSplineHeading(blockPose, tangent)
                        .build()
                )

                .stopAndAdd(robot.getCollectingSystem().extendAndCollectAction(EXTEND_POSITION))

                // retract and go to deposit position at same time
                .stopAndAdd(new ParallelAction(
                        robot.getExtension().retractAction(),
                        lineToPos(Auto.getPositionVector(yellowBlockParams.getBeginPose()), Auto.getHeading(yellowBlockParams.getBeginPose()))
                ))
                .build();
    }
}
