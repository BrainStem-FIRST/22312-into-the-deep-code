package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpwRoadrunner")
public class Tele extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private BrainSTEMRobotTele robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BrainSTEMRobotTele(this.hardwareMap, this.telemetry, this);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            updateRobotControls();

            robot.update();
            telemetry.addData("drivetrain encoder", robot.driveTrain.getTick());
        }
    }

    private void updateRobotControls() {

        // drivetrain
        updateDriveTrainControls();

        // collector
        updateCollectorControls();
    }

    private void updateDriveTrainControls() {
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y * -1;
        double rightStickX;
        double threshold = 0.1F;
        if (Math.abs(gamepad1.right_stick_x) > threshold) {
            if (gamepad1.right_stick_x < 0) {
                rightStickX = (gamepad1.right_stick_x * gamepad1.right_stick_x * -1 * (4.0 / 5.0) - (1.0 / 5.0));
            } else {
                rightStickX = (gamepad1.right_stick_x * gamepad1.right_stick_x * (4.0 / 5.0) + (1.0 / 5.0));
            }
        } else {
            rightStickX = 0;
        }
        if ((Math.abs(gamepad1.left_stick_y) > threshold) || (Math.abs(gamepad1.left_stick_x) > threshold) || Math.abs(gamepad1.right_stick_x) > threshold) {
            //Calculate formula for mecanum drive function
            double addValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) + leftStickX * Math.abs(leftStickX))))) / 100;
            double subtractValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) - leftStickX * Math.abs(leftStickX))))) / 100;


            //Set motor speed variables
            robot.driveTrain.setDTMotorPowers((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
        } else {
            stop();
        }

        telemetry.addData("front left tick: ", robot.driveTrain.getTick());
        telemetry.update();
    }

    private void updateCollectorControls() {
        if (gamepad1.a) {
            robot.collector.takeInBlock();
        }
        else if (gamepad1.b) {
            robot.collector.spitOutBlock();
        }
    }
}
