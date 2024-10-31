package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.Arm;

public class ArmTele extends Arm {
    public ArmTele(HardwareMap hw, Telemetry telemetry, AllianceColor allianceColor) {
        super(hw, telemetry, allianceColor);
    }

    public void update() {
        if(getGoalState() != null) {
            // checking for end of state transition
            if(Math.abs(getArmServo().getPosition() - getPrepStatePositions().get(getGoalState()) <= ERROR))
                goalStateReached();
            else
                getArmServo().setPosition(getPrepStatePositions().get(getGoalState()));
        }
    }
}
