package org.firstinspires.ftc.teamcode.auto;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

@Autonomous
public class TestAuto extends LinearOpMode {
    BrainSTEMRobot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED);
    }
}
