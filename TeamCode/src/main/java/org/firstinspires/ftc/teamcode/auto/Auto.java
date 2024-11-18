package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;


public abstract class Auto extends LinearOpMode {

    public static Vector2d getPositionVector(Pose2d pose) {
        return new Vector2d(pose.position.x, pose.position.y);
    }
    public static double getHeading(Pose2d pose) {
        return Math.atan(pose.position.y / pose.position.x);
    }

    private final AllianceColor allianceColor;

    protected BrainSTEMRobot robot;

    public Auto(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(hardwareMap, telemetry, allianceColor);
        robot.setup();

        waitForStart();

        runAuto();
    }

    // moves in a straight line to a target position and heading
    public Action lineToPos(Vector2d pos, double heading) {
        return new ParallelAction(
                robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)
                        .lineToX(pos.x)
                        .build(),
                robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)
                        .lineToY(pos.y)
                        .build(),
                robot.getDriveTrain().actionBuilder(robot.getDriveTrain().pose)
                        .turnTo(heading)
                        .build()
        );
    }

    public abstract void runAuto();

}
