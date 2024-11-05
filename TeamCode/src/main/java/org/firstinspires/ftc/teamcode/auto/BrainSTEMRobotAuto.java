package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.util.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.gamepadInput.Input;

public class BrainSTEMRobotAuto extends BrainSTEMRobot {

    public LiftAuto lift;
    public CollectorAuto collector;

    public BrainSTEMRobotAuto(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {

        super(hwMap, telemetry, allianceColor);

        lift = new LiftAuto(hwMap, telemetry, allianceColor, this);
        collector = new CollectorAuto(hwMap, telemetry, allianceColor, this);
    }

    public void resetAllEncoders() {

    }
}
