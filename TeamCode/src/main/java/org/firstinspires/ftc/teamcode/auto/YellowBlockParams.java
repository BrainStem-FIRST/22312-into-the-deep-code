package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class YellowBlockParams {
    private final Pose2d beginPose;
    private final Vector2d blockOnePos;
    private final double blockOneHeading;
    private final Vector2d blockTwoPos;
    private final double blockTwoHeading;
    private final Pose2d blockThreePose;
    private final double blockThreeTangent;

    public YellowBlockParams(Pose2d beginPose, Vector2d blockOnePos, double blockOneHeading, Vector2d blockTwoPos, double blockTwoHeading, Pose2d blockThreePose, double blockThreeTangent) {
        this.beginPose = beginPose;

        this.blockOnePos = blockOnePos;
        this.blockOneHeading = blockOneHeading;

        this.blockTwoPos = blockTwoPos;
        this.blockTwoHeading = blockTwoHeading;

        this.blockThreePose = blockThreePose;
        this.blockThreeTangent = blockThreeTangent;
    }
    public Pose2d getBeginPose() {
        return beginPose;
    }

    public Vector2d getBlockOnePos() {
        return blockOnePos;
    }
    public double getBlockOneHeading() {
        return blockOneHeading;
    }
    public Vector2d getBlockTwoPos() {
        return blockTwoPos;
    }
    public double getBlockTwoHeading() {
        return blockTwoHeading;
    }
    public Pose2d getBlockThreePose() {
        return blockThreePose;
    }

    public double getBlockThreeTangent() {
        return blockThreeTangent;
    }
}
