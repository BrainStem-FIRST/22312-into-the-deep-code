package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;

@Autonomous
public class AutoSpecimenRedTeam extends AutoSpecimen {

    public AutoSpecimenRedTeam() {
        // TODO: find values for specimen red
        super(AllianceColor.BLUE, new AllianceColorParams(
                new Pose2d(0, 0, 0),
                new Pose2d(0, 0, 0),
                0,
                new Pose2d(0, 0, 0),
                0,
                new Pose2d(0, 0, 0),
                0,
                new Vector2d(0, 0),
                0,
                new Vector2d(0, 0)
        ));
    }

    @Override
    public void runAuto() {

    }
}
