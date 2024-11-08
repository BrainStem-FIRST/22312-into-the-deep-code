package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.BrainSTEMRobotAuto;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

public abstract class Auto extends LinearOpMode {

    private final AllianceColor allianceColor;

    protected BrainSTEMRobot robot;

    public Auto(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public abstract Pose2d getBeginPose();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(hardwareMap, telemetry, allianceColor);
        robot.setup();

        waitForStart();
        runAuto();
    }

    public abstract void runAuto();

}
