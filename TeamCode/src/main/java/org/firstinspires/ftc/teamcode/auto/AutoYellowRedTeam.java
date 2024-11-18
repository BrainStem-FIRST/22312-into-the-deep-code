package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

public class AutoYellowRedTeam extends AutoYellow {
    public AutoYellowRedTeam() {
        super(new YellowBlockParams(
                new Pose2d(-63, -85, Math.PI*1.5),
                new Vector2d(-24, -42.5),
                Math.PI * 1.5,
                new Vector2d(-24, -42.5),
                Math.PI * 1.5,
                new Pose2d(-24, -42.5, Math.PI * 0.75),
                Math.PI * 0.75
        ));
    }
}
