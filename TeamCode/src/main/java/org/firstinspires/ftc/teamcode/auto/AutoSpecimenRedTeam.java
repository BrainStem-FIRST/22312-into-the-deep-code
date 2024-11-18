package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;

@Autonomous
public class AutoSpecimenRedTeam extends Auto {

    public AutoSpecimenRedTeam() {
        super(AllianceColor.RED);
    }

    @Override
    public Pose2d getBeginPose() {
        return null;
    }

    @Override
    public void runAuto() {

    }
}
