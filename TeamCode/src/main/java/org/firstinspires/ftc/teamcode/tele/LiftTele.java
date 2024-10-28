package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Lift;

public class LiftTele extends Lift {



    public LiftTele(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);
    }

    private void raiseLift() {
        getLiftMotor().setPower(0.3);
    }
    private void dropLift() {
        getLiftMotor().setPower(-0.3);
    }
    private void holdLift() {
        getLiftMotor().setPower(0);
    }

    public void update() {
        if (getGoalState() != null) {
            // checking if done with transition
            double dif = getStatePositions().get(getGoalState()) - getStatePositions().get(getState());
            if (dif > MOTOR_THRESHOLD)
                raiseLift();
            else if (dif < -MOTOR_THRESHOLD)
                dropLift();
            else {
                setState(getGoalState());
                setGoalState(null);
                holdLift();
            }
        }
        else
            holdLift();
    }
}

