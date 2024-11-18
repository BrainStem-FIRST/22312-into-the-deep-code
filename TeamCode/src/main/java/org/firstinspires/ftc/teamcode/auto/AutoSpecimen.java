package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Extension;

public class AutoSpecimen extends Auto {
    private final AllianceColorParams allianceColorParams;
    public AutoSpecimen(AllianceColor allianceColor, AllianceColorParams allianceColorParams) {
        super(allianceColor);
        this.allianceColorParams = allianceColorParams;

    }
    @Override
    public void runAuto() {
        Actions.runBlocking(
            new SequentialAction(
                hangSpecimenInitial(),

                obtainSpecimen(allianceColorParams.getBlockOnePos(), allianceColorParams.getBlockOneTangent()),
                hangSpecimen(),

                obtainSpecimen(allianceColorParams.getBlockTwoPos(), allianceColorParams.getBlockTwoTangent()),
                hangSpecimen(),

                obtainSpecimen(allianceColorParams.getBlockThreePos(), allianceColorParams.getBlockThreeTangent()),
                hangSpecimen()
            )
        );
    }

    // pre-condition: ASSUMES ROBOT IS AT STARTING POSITION WITH SPECIMEN IN TROUGH
    private Action hangSpecimenInitial() {
        return robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)

                .stopAndAdd(new ParallelAction(
                        lineToPos(allianceColorParams.getHangPos(), allianceColorParams.getHangTangent()),
                        robot.getLiftingSystem().getSetupHighSpecimenRamInitial()
                ))

                .stopAndAdd(new ParallelAction(
                        robot.getLiftingSystem().getRamHighSpecimen(),
                        lineToPos(allianceColorParams.getHangAfterPos(), allianceColorParams.getHangTangent())
                ))

                .build();
    }
    private Action hangSpecimen() {
        return robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)

                .stopAndAdd(new ParallelAction(
                        lineToPos(allianceColorParams.getHangPos(), allianceColorParams.getHangTangent()),
                        robot.getLiftingSystem().getSetupHighSpecimenRam()
                ))

                .stopAndAdd(new ParallelAction(
                        robot.getLiftingSystem().getRamHighSpecimen(),
                        lineToPos(allianceColorParams.getHangAfterPos(), allianceColorParams.getHangTangent())
                ))

                .build();
    }

    // pre-condition: ASSUMES ROBOT JUST HUNG, NOTHING IS RESET YET
    private Action obtainSpecimen(Pose2d blockPose, double tangent) {
        return robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)

            .stopAndAdd(new ParallelAction(
                    robot.getLiftingSystem().getResetHighSpecimenRam(),
                    robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)
                        .splineToSplineHeading(blockPose, tangent).build()
            ))

            .stopAndAdd(robot.getCollectingSystem().extendAndCollectAction(Extension.SHORT_EXTEND_POSITION))

            .stopAndAdd(new ParallelAction(
                new SequentialAction(
                        robot.getCollectingSystem().retractAction(),
                        robot.getLiftingSystem().transferBlock()
                ),
                robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)
                    .splineToSplineHeading(allianceColorParams.getBeginPose(), allianceColorParams.getBeginPose().heading).build()
            ))

            .stopAndAdd(robot.getLiftingSystem().getDropOffBlock())
            .stopAndAdd(new TimedAction() {
                @Override
                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    updateFramesRunning();
                    return getTime() > 1;
                }
            })
            .stopAndAdd(robot.getLiftingSystem().getPickUpSpecimen())

            .build();
    }
}
