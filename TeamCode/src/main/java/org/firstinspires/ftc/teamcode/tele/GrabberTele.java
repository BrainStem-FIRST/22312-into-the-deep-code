package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Grabber;

public class GrabberTele extends Grabber {
    public GrabberTele(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);
    }

    public void update() {
        // meaning in transition
        if(getGoalState() != getState())
            // checking if done transitioning
            if(getGrabServo().getPosition() == getStatePositions().get(getGoalState()))
                setState(getGoalState());
            else
                // technically only need to call this once, but o well
                getGrabServo().setPosition(getStatePositions().get(getGoalState()));
    }
}
