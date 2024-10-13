package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.tele.CollectorTele;
import org.firstinspires.ftc.teamcode.tele.ExtensionTele;

public class CollectingExtendingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        CollectorTele collector = new CollectorTele(hardwareMap, telemetry);
        ExtensionTele extension = new ExtensionTele(hardwareMap, telemetry);

        while (opModeIsActive()) {

            if (gamepad1.a && extension.isRetracted()) {
                extension.setState(Extension.State.EXTENDING);
                collector.setHingeState(Collector.HingeState.HINGING_DOWN);
                collector.setCollectState(Collector.CollectState.COLLECTING);
            }

            if (extension.isExtended() && collector.getCollectState() == Collector.CollectState.FULL) {
                extension.setState(Extension.State.RETRACTING);
                collector.setHingeState(Collector.HingeState.HINGING_UP);
            }

            extension.update();
            collector.update();
        }
    }
}
