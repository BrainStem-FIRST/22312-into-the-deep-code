package org.firstinspires.ftc.teamcode.auto;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;

@Autonomous
public class AutoYellowRedTeam extends Auto {

    public static int EXTEND_POSITION = 200;
    public AutoYellowRedTeam() {
        super(AllianceColor.BLUE);
    }

    @Override
    public Pose2d getBeginPose() {
        return new Pose2d(-63, -85, Math.PI*1.5);
    }

    @Override
    public void runAuto() {
        runBlocking(
                blockOneTrajectory()
        );
    }

    public Action blockOneTrajectory() {
        return robot.getDriveTrain().actionBuilder(getBeginPose())
                .stopAndAdd(robot.getLiftingSystem().depositHigh())
                .splineToConstantHeading(new Vector2d(-24, -42.5), Math.PI * 1.5)
                .stopAndAdd(robot.getCollectingSystem().extendAndCollectAction(EXTEND_POSITION))
                .splineToConstantHeading(getBeginPose().position, getBeginPose().heading.toDouble())
                .build();
    }
}
