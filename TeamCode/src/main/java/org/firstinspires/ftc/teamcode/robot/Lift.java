package org.firstinspires.ftc.teamcode.robot;

import java.util.HashMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends LiftSubsystem {
    private final DcMotorEx liftMotor;

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor, 10);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setPrepStatePositions(0, 3, 5, 10, 6);
        setExecStatePositions(0, 3, 5, 10, 9);
    }
    @Override
    public boolean executeCurrentState() {
        int pos = (int) (double) getExecStatePositions().get(getCurState());
        liftMotor.setTargetPosition(pos);
        return liftMotor.getCurrentPosition() == pos;
    }

    public DcMotorEx getLiftMotor() {
        return liftMotor;
    }
}

