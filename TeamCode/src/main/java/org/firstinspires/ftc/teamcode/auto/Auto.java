package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.ftc.Actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.tele.BrainSTEMRobotTele;

public abstract class Auto extends LinearOpMode {

    public Pose2d beginPose;
    private AllianceColor allianceColor;

    public Auto(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobotAuto robot = new BrainSTEMRobotAuto(hardwareMap, telemetry, this, allianceColor);

        waitForStart();

        robot.drive.pose = beginPose;
        runAuto();
    }

    public abstract void runAuto();

}
