package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.TickChecker;


public class Extension extends SubsystemTele {
    private DcMotorEx extensionMotor;
    private ServoImplEx hingeServo;

    private enum States {
        IN, LOW, MED, HIGH
    }
    private States state = States.LOW;

    public Extension(Telemetry telemetry, HardwarMap hwMap) {
        super(telemetry, hwMap);
        extensionMotor = new CachingMotor(this.hwMap.get(DcMotorEx.class, "extensionMotor"));
        hingeServo = new CachingServo(this.hwMap.get(ServoImplEx.class, "collectorHingeServo"), MIN, MAX); // MIN AND MAX NOT INITIZALIZED ANYWHERE
    }

    @Override
    public void checkState() {
        switch (state) {
            case IN:
                inTransition();
                break;
            case LOW:
                lowTransition();
                break;
            case MED:
                medTransition();
                break;
            case HIGH:
                highTransition();
                break;
        }
    }

    private void inTransition() {

    }
    private void lowTransition() {

    }
    private void medTransition() {

    }
    private void highTransition() {

    }
}