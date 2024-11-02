package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Collector;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "CollectingExtendingTest")
public class CollectingExtendingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        CollectorTele collector = new CollectorTele(hardwareMap, telemetry, AllianceColor.BLUE);
        ExtensionTele extension = new ExtensionTele(hardwareMap, telemetry, AllianceColor.BLUE);
        collector.setHingeServoPosition(Collector.HINGE_UP_POSITION);

        while (opModeIsActive()) {

            telemetry.addData("gamepad.a", gamepad1.a);
            telemetry.addData("gamepad.b", gamepad1.b);
            telemetry.addData("collector spindle motor power", collector.getSpindleMotor().getPower());
            telemetry.addData("collector hinge servo position", collector.getHingeServo().getPosition());
            telemetry.update();
        }
    }
}
