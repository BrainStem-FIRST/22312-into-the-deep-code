package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftTele extends Lift {

    DcMotorEx liftMotor;

    enum LiftStates {
        UP, DOWN, STATIC
    };

    LiftStates liftState;

    public LiftTele(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftState = LiftStates.STATIC;
    }

    private void raiseLift() {
        liftMotor.setPower(0.3);
    }
    private void dropLift() {
        liftMotor.setPower(-0.3);
    }
    private void holdLift() {
        liftMotor.setPower(0);
    }

    private void updateState() {
        switch(liftState) {
            case UP:
                raiseLift();
                break;
            case DOWN:
                dropLift();
                break;
            case STATIC:
                holdLift();
                break;
        }
    }

    public void update() {
        updateState();
    }
}

