package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import org.firstinspires.ftc.teamcode.util.CachingServo;
import org.firstinspires.ftc.teamcode.util.CachingMotor;
import org.firstinspires.ftc.teamcode.util.TickChecker;


// state mechanics: once collector/spit state initiated, will power until time reached, them automatically turn off
// do not need to turn motor off outside of this class

// need to add color sensor; could represent the one detecting block in grabber
// the collecting sequence should end only when color sensor detects block over extended period
// collecting sequence:
// collect motor spins in, and servo slowly turns inward and rapidly resets to try and match block
// if block is good, signal for de-extension
// if block not good, spin wheels in opposite direction and run collect motor in opposite direction
// then would need to adjust robot (and potentially extension) position to search new area for block

public class CollectorTele extends Collector {
    // need servo for grabber (wheels)
    // need dcExMotor for rollers
    private DcMotorEx rollerMotor;
    private ServoImplEx grabberServo;

    // arbitrary numbers to judge against encoder ticks
    private TickChecker collectTickChecker = new TickChecker(5000);
    private double collectPower = 0.8;
    private TickChecker spitTickChecker = new TickChecker(1000);
    private double spitPower = -0.8;

    private enum States {
        COLLECTING, SPITTING, OFF
    }
    private States state = States.OFF;

    public CollectorTele(HardwareMap hwMap, Telemetry telemetry) {
        super(hwMap, telemetry);

        this.rollerMotor = new CachingMotor(this.hwMap.get(DcMotorEx.class, "roller"));
        this.grabberServo = new CachingServo(this.hwMap.get(ServoImpleEx.class, "grabber"), MIN, MAX); // MIN AND MAX HAVE NOT BEEN INITIALIZED YET
    }

    // state setters
    public setCollect() {
        state = States.COLLECTING;
    }

    public setSpit() {
        state = States.SPITTING;
    }

    public setOff() {
        state = States.OFF;
    }

    @Override
    public void checkState() {
        switch (state) {
            case COLLECTING:
                collect();
                break;
            case SPITTING:
                spit();
                break;
            case OFF:
                off();

                // internal functionality
                private void collect () {
                if (collectTickChecker.valid()) {
                    rollerMotor.setPower(collectPower);
                } else {
                    setOff();
                }
            }
            private void spit () {
                grabberServo.setPosition(0.99);
                if (spitTickChecker.valid()) {
                    rollerMotor.setPower(spitPower);
                } else {
                    setOff();
                }
            }
            private void off () {
                rollerMotor.setPower(0);
            }
        }
    }
}




