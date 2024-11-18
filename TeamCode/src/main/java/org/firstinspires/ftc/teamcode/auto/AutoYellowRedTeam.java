package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;

@Autonomous
public class AutoYellowRedTeam extends AutoYellow {
    public AutoYellowRedTeam() {
        super(AllianceColor.RED, new YellowBlockParams(
                new Pose2d(-48, -85, Math.PI*1.5), // begin position

                new Vector2d(-24, -42.5), // destination and heading for block one
                Math.PI * 1.5,

                new Vector2d(-60, -42.5), // destination and heading for block two
                Math.PI * 1.5,

                new Pose2d(-64, -40, Math.PI * 0.75), // path and heading for block three
                Math.PI * 0.75
        ));
    }
}
