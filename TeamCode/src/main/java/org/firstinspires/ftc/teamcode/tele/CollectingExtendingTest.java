package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Collector;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CollectingExtendingTest")
public class CollectingExtendingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        CollectorTele collector = new CollectorTele(hardwareMap, telemetry);
        //ExtensionTele extension = new ExtensionTele(hardwareMap, telemetry);
        collector.setCollectState(Collector.CollectState.EMPTY);
        collector.setHingeState(Collector.HingeState.UP);
        collector.setHingeServoPosition(Collector.HINGE_UP_POSITION);

        while (opModeIsActive()) {

            telemetry.addData("gamepad.a", gamepad1.a);
            telemetry.addData("gamepad.b", gamepad1.b);
            telemetry.addData("collector collect state", collector.getCollectState());
            telemetry.addData("collector hinge state", collector.getHingeState());
            telemetry.addData("collector spindle motor power", collector.getSpindleMotor().getPower());
            telemetry.addData("collector hinge servo position", collector.getHingeServo().getPosition());
            telemetry.update();

            if (gamepad1.a
                    && collector.getCollectState() == Collector.CollectState.EMPTY
                    && collector.getHingeState() == Collector.HingeState.UP) {
                //extension.setState(Extension.State.EXTENDING);
                // collector automatically sets hingeState to DOWN when it finishes hinging
                collector.setHingeState(Collector.HingeState.HINGING_DOWN);
                // collector keeps collecting until block is in chamber, then it sets collectState to FULL_MAX
                collector.setCollectState(Collector.CollectState.COLLECTING);
            }

            //if (extension.isExtended() && collector.getCollectState() == Collector.CollectState.FULL_MAX) {
            //if (gamepad1.b && collector.getHingeState() == Collector.HingeState.DOWN) {
            // keep spindle motors spinning at full speed when hinge position is down
            // only slows down spindle motors when hinge position is up
            if (collector.getCollectState() == Collector.CollectState.FULL_MAX) {
                //extension.setState(Extension.State.RETRACTING);
                collector.setHingeState(Collector.HingeState.HINGING_UP);
            }

            // manual reset (for now)
            if (gamepad1.x) {
                collector.reset();
            }


            //extension.update();
            collector.update();
        }
    }
}
