package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;

public abstract class Auto extends LinearOpMode {

    private final AllianceColor allianceColor;

    public Auto(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public abstract Pose2d getBeginPose();

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobotAuto robot = new BrainSTEMRobotAuto(hardwareMap, telemetry, this, allianceColor, this.gamepad1);

        waitForStart();

        robot.drive.pose = getBeginPose();
        //robot.resetAllEncoders();
        runAuto();
    }

    public abstract void runAuto();

}
