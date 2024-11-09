package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Drawing;

public class PIDLocalizationTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        PIDDrivetrain drive = new PIDDrivetrain(hardwareMap, telemetry, new Pose2D(DistanceUnit.INCH,0,0,
                AngleUnit.RADIANS, 0));

        waitForStart();

        while (opModeIsActive()) {

            drive.updatePoseEstimate();
            driveTrainControl(drive);

            double x = drive.getCurrentPose().getX(DistanceUnit.INCH);
            double y = drive.getCurrentPose().getY(DistanceUnit.INCH);
            double heading = drive.getCurrentPose().getHeading(AngleUnit.DEGREES);
            Pose2d rrPose = new Pose2d(x, y, Math.toRadians(heading));

            telemetry.addData("x", x);
            telemetry.addData("y", y);
            telemetry.addData("heading (deg)", heading);
            telemetry.update();

            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setStroke("#3F51B5");
            Drawing.drawRobot(packet.fieldOverlay(), rrPose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    private void driveTrainControl(PIDDrivetrain drive) {
        double rightStickX;
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y * -1;
        double threshold = 0.2;

        if (Math.abs(gamepad1.right_stick_x) > threshold) {
            if (gamepad1.right_stick_x < 0)
                rightStickX = gamepad1.right_stick_x * gamepad1.right_stick_x * -1 * (4.0 / 5.0) - (1.0 / 5.0);
            else
                rightStickX = gamepad1.right_stick_x * gamepad1.right_stick_x * (4.0 / 5.0) + (1.0 / 5.0);
        } else
            rightStickX = 0;

        if ((Math.abs(gamepad1.left_stick_y) > threshold) || (Math.abs(gamepad1.left_stick_x) > threshold) || Math.abs(gamepad1.right_stick_x) > threshold) {
            //Calculate formula for mecanum drive function
            double addValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) + leftStickX * Math.abs(leftStickX))))) / 100;
            double subtractValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) - leftStickX * Math.abs(leftStickX))))) / 100;

            //Set motor speed variables
            drive.setDrivePower((addValue + rightStickX)/1.05, (subtractValue + rightStickX)/1.05, (subtractValue - rightStickX)/1.05, (addValue - rightStickX)/1.05);
        } else {
            drive.stop();
        }
    }
}