package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;


public class AllianceColorParams {
    private final Pose2d beginPose;
    private final Pose2d blockOnePos;
    private final double blockOneTangent;
    private final Pose2d blockTwoPos;
    private final double blockTwoTangent;
    private final Pose2d blockThreePos;
    private final double blockThreeTangent;
    private final Vector2d hangPos;
    private final double hangTangent;
    private final Vector2d hangAfterPos;

    public AllianceColorParams(Pose2d beginPose, Pose2d blockOnePos, double blockOneTangent, Pose2d blockTwoPos, double blockTwoTangent, Pose2d blockThreePos, double blockThreeTangent, Vector2d hangPos, double hangTangent, Vector2d hangAfterPos) {
        this.beginPose = beginPose;
        this.blockOnePos = blockOnePos;
        this.blockOneTangent = blockOneTangent;
        this.blockTwoPos = blockTwoPos;
        this.blockTwoTangent = blockTwoTangent;
        this.blockThreePos = blockThreePos;
        this.blockThreeTangent = blockThreeTangent;
        this.hangPos = hangPos;
        this.hangTangent = hangTangent;
        this.hangAfterPos = hangAfterPos;
    }
    public Pose2d getBeginPose() {
        return beginPose;
    }
    public Pose2d getBlockOnePos() {
        return blockOnePos;
    }
    public Pose2d getBlockTwoPos() {
        return blockTwoPos;
    }
    public Pose2d getBlockThreePos() {
        return blockThreePos;
    }
    public double getBlockOneTangent() {
        return blockOneTangent;
    }
    public double getBlockTwoTangent() {
        return blockTwoTangent;
    }
    public double getBlockThreeTangent() {
        return blockThreeTangent;
    }
    public Vector2d getHangPos() {
        return hangPos;
    }
    public double getHangTangent() {
        return hangTangent;
    }
    public Vector2d getHangAfterPos() {
        return hangAfterPos;
    }
}
