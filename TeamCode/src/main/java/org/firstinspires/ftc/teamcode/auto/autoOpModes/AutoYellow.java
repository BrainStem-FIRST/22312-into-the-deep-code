package org.firstinspires.ftc.teamcode.auto.autoOpModes;

import static com.acmerobotics.roadrunner.ftc.Actions.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;

public abstract class AutoYellow extends Auto {

    public AutoYellow(AllianceColor allianceColor) {
        super(allianceColor);
    }

    public abstract int[][] getYellowBlockPositions();

    @Override
    public void runAuto() {
        runBlocking(
                new SequentialAction(

                )
        );
    }

    public Action makeDriveTrajectory(int[] blockPosition) {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                robot.getDriveTrain().actionBuilder(getBeginPose())
                        .build();
                return false;
            }
        };
    }
}
