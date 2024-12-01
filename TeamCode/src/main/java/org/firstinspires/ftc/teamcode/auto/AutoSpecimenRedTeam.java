package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;

@Autonomous
@Config
public class AutoSpecimenRedTeam extends LinearOpMode {
    //public static Pose2d BLOCK_ONE_POSE = new Pose2d()
        /*
        // TODO: find values for specimen red
        super(AllianceColor.BLUE, new AllianceColorParams(
                new Pose2d(31.5, -64.5, Math.PI/2),
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
         */
    public static class Params {
        public double beginX = 31.5, beginY = -64.5, beginA = Math.PI/2;
        public double hangX = 0, hangY = -21.8, hangA = Math.PI/2;
    }
    Params PARAMS = new Params();

    @Override
    public void runOpMode() throws InterruptedException {
        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, AllianceColor.RED);
        PinpointDrive drive = robot.getDriveTrain();

        Pose2d BEGIN_POSE = new Pose2d(PARAMS.beginX, PARAMS.beginY, PARAMS.beginA);
        //Pose2d HANG_MIDDLE_POSE = new Pose2d(PARAMS, -21.8, Math.PI/2);

        waitForStart();

        Actions.runBlocking(
            drive.actionBuilder(BEGIN_POSE)
                .splineTo(new Vector2d(PARAMS.hangX, PARAMS.hangY), PARAMS.hangA)
                .build());

    }
}
