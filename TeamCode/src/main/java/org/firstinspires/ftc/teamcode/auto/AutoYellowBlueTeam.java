package org.firstinspires.ftc.teamcode.auto;

import static com.acmerobotics.roadrunner.ftc.Actions.runBlocking;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;

public class AutoYellowBlueTeam extends Auto {

    public static int YELLOW_1_EXTEND_TICK = 250;
    public AutoYellowBlueTeam() {
        super(AllianceColor.BLUE);
    }

    @Override
    public Pose2d getBeginPose() {
        return null;
    }

    @Override
    public void runAuto() {
        runBlocking(
                robot.getCollectingSystem().extendAndCollectAction(YELLOW_1_EXTEND_TICK)
        );
    }
}
