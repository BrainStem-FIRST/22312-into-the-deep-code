package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;

@Autonomous
public class AutoYellowRedTeam extends AutoYellow {
    public AutoYellowRedTeam() {
        super(AllianceColor.RED, new YellowBlockParams(
                new Pose2d(-63, -85, Math.PI*1.5),
                new Vector2d(-24, -42.5),
                Math.PI * 1.5,
                // TODO: find block positions and headings for two and three
                new Vector2d(-24, -42.5),
                Math.PI * 1.5,
                new Pose2d(-24, -42.5, Math.PI * 0.75),
                Math.PI * 0.75
        ));
    }
}
