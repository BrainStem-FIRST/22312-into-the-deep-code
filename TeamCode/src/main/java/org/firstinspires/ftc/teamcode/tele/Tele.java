package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOpwRoadrunner")
public class Tele extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private BrainSTEMRobot robot;

    private AllianceColor allianceColor;

    public Tele(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new BrainSTEMRobot(this.hardwareMap, this.telemetry, this, allianceColor);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        // TODO - idk if this works or not w rev
        long currentTime = System.currentTimeMillis();
        long prevTime;
        double dt;

        while (opModeIsActive()) {
            prevTime = currentTime;
            currentTime = System.currentTimeMillis();
            dt = (currentTime - prevTime) / 1000.;

            listenForRobotControls();

            robot.update(dt);
        }
    }

    private void listenForRobotControls() {
        listenForDriveTrainInput();
        listenForCollectionInput();
        listenForLiftInput();
    }

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
        // TODO - this code is probably very buggy b/c I haven't actually tested it with the robot
        // extend and retract collector
        if (gamepad1.a)
            if (!robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.EXTENDING))
                robot.getCollectingSystem().getStateManager().tryEnterState(CollectingSystem.StateType.RETRACTING);
    }
    private void listenForLiftInput() {

    }
}
