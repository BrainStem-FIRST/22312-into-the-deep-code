package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;

public class BrainSTEMRobotTele {
    // Don't touch these
    public Telemetry telemetry;
    public OpMode opMode;

    public DriveTrainTele driveTrain;
    public LiftTele lift;
    public ExtensionTele extension;
    public CollectorTele collector;
    public GrabberTele grabber;
    public BrainSTEMRobotTele(HardwareMap hwMap, Telemetry telemetry, OpMode opMode) {

        this.telemetry = telemetry;
        this.opMode = opMode;

        driveTrain = new DriveTrainTele(hwMap, telemetry);
        //lift = new LiftTele(hwMap, telemetry);
        //extension = new ExtensionTele(hwMap, telemetry);
        collector = new CollectorTele(hwMap, telemetry);
    }

    public void update() {
        //extension.update();
        collector.update();
    }

    public void extendAndCollect() {
        /*if (extension.isRetracted()) {
            extension.setState(Extension.State.EXTENDING);
            // assuming robot is flush with barrier
            collector.setHingeState(Collector.HingeState.HINGING_DOWN);
            collector.setCollectState(Collector.CollectState.COLLECTING);
        }*/
        if (collector.getCollectState() == Collector.CollectState.EMPTY && collector.getHingeState() == Collector.HingeState.UP) {
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
    }
}
