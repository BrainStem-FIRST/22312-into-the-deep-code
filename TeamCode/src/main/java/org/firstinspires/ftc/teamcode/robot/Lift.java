package org.firstinspires.ftc.teamcode.robot;

import java.util.HashMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Lift extends LiftSubsystem {
    private final DcMotorEx liftMotor;
    public final int DESTINATION_THRESHOLD = 10;

    public Lift(HardwareMap hwMap, Telemetry telemetry, AllianceColor allianceColor) {
        super(hwMap, telemetry, allianceColor);

        liftMotor = (DcMotorEx) hwMap.dcMotor.get("LiftMotor");
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        setPrepStatePositions(0, 5, 10, 4);
        setExecStatePositions(0, 9, 10, 4);
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

