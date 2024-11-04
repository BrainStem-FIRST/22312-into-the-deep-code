package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleMain")
public class Tele extends LinearOpMode {

    private BrainSTEMRobot robot;


    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, this, AllianceColor.BLUE);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        robot.setup();

        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;
        double interval = 1000/60.;
        double timeSinceLastUpdate = 0;

        while (opModeIsActive()) {
            prevTime = currentTime;
            currentTime = System.currentTimeMillis();
            dt = (currentTime - prevTime) / 1000.;
            timeSinceLastUpdate += dt;

            //if (timeSinceLastUpdate < interval)
            //    continue;
            //timeSinceLastUpdate -= interval;

            listenForCollectionInput();
            robot.update(dt);
            telemetry.update();
        }
    }

    private void listenForRobotControls() {
        telemetry.addData("inside listen for robot controls", "");
        //listenForDriveTrainInput();
        listenForCollectionInput();
    }

    // without roadrunner
    private void listenForDriveTrainInput() {

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
            robot.getDriveTrain().setDrivePower((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
        } else
            robot.getDriveTrain().stop();
    }


    private void listenForCollectionInput() {
        // a extends and retracts collector
        telemetry.addData("a", gamepad1.a);
        telemetry.addData("collector hinge encoder tick", robot.getCollector().getHingeServo().getPosition());
        telemetry.addData("collecting system state, ", robot.getCollectingSystem().getStateManager().getActiveStateType());
        telemetry.addData("extension state, ",  robot.getExtension().getStateManager().getActiveStateType());
        telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
        telemetry.addData("hinge servo position", robot.getCollector().getHingeServo().getPosition());

        if (gamepad1.a) {
            // a serves to both extend and retract, so if robot cannot extend, it tries to retract
            if (!robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.EXTENDING)) {
                //robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);
            }
            //robot.getCollector().getHingeServo().setPosition(Collector.HINGE_DOWN_POSITION);
        }
    }
}
