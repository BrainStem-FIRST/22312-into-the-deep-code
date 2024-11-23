package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.PIDController;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ExtensionTestingTele")
public class ExtensionTestingTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        PIDController pid = new PIDController(0.01, 0, 0);
        boolean inPIDMode = true;
        pid.setTarget(Extension.MIN_POSITION);

        while(opModeIsActive()) {
            if (gamepad1.a)
                Subsystem.setMotorPower(motor, 0.3);
            else if (gamepad1.b)
                Subsystem.setMotorPower(motor, pid.update(motor.getCurrentPosition()));
            else if(gamepad1.x)
                Subsystem.setMotorPower(motor, Range.clip(0.01 * (Extension.MIN_POSITION - motor.getCurrentPosition()), -1, 1));
            else
                Subsystem.setMotorPower(motor, 0);
            telemetry.addData("extension encoder", motor.getCurrentPosition());
            telemetry.addData("extension motor power", motor.getPower());
            telemetry.addData("manual pid value", Range.clip(0.01 * (Extension.MIN_POSITION - motor.getCurrentPosition()), -1, 1));
            telemetry.update();
        }
    }
}

