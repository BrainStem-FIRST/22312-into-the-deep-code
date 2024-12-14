package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Subsystem;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "DriveTrainTestTele")
public class DriveTrainTele extends LinearOpMode {
    private DcMotorEx fl, fr, bl, br;
    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        br = hardwareMap.get(DcMotorEx.class, "BR");

        waitForStart();
        ElapsedTime timer = new ElapsedTime();
        while(opModeIsActive()) {
            listenForDriveTrainInputOld();
            telemetry.addData("timer (to ensure opmode is continuously running", timer.seconds());
            telemetry.update();
        }
    }
    private void setMotorPowers(double fl, double fr, double bl, double br) {
        Subsystem.setMotorPower(this.fl, fl);
        Subsystem.setMotorPower(this.fr, fr);
        Subsystem.setMotorPower(this.bl, bl);
        Subsystem.setMotorPower(this.br, br);
    }
    private void listenForDriveTrainInputOld() {
        telemetry.addData("listening for drive train input (not using roadrunner)", "");
        // drivetrain
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = gamepad1.left_stick_y * -1;
        double rightStickX;
        double threshold = 0.1F;
        if (Math.abs(gamepad1.right_stick_x) > threshold) {
            if (gamepad1.right_stick_x < 0) {
                rightStickX = (gamepad1.right_stick_x * gamepad1.right_stick_x * -1 * (4.0 / 5.0) - (1.0 / 5.0));
            } else {
                rightStickX = (gamepad1.right_stick_x * gamepad1.right_stick_x * (4.0 / 5.0) + (1.0 / 5.0));
            }
        } else {
            rightStickX = 0;
        }
        if ((Math.abs(gamepad1.left_stick_y) > threshold) || (Math.abs(gamepad1.left_stick_x) > threshold) || Math.abs(gamepad1.right_stick_x) > threshold) {
            //Calculate formula for mecanum drive function
            double addValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) + leftStickX * Math.abs(leftStickX))))) / 100;
            double subtractValue = (double) (Math.round((100 * (leftStickY * Math.abs(leftStickY) - leftStickX * Math.abs(leftStickX))))) / 100;


            //Set motor speed variables
            setMotorPowers((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
        } else {
            setMotorPowers(0, 0, 0, 0);
        }
    }
}
