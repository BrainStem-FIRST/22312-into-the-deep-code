package org.firstinspires.ftc.teamcode.auto.autoOpModes;


import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.BrainSTEMRobotAuto;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public abstract class Auto extends LinearOpMode {

    private final AllianceColor allianceColor;

    protected BrainSTEMRobotAuto robot;

    public Auto(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public abstract Pose2d getBeginPose();

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobotAuto(hardwareMap, telemetry, allianceColor);

        waitForStart();

        robot.resetAllEncoders();
        runAuto();
    }

    public abstract void runAuto();

}
