package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.Input;
import org.firstinspires.ftc.teamcode.util.PIDController;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "LiftTuneTele")
public class LiftTuneTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Input input = new Input(gamepad1, gamepad2);
        // time for tracking servo transition times
        ElapsedTime transitionTime = new ElapsedTime();
        double lastTransitionDuration = 0;

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();

        PIDController liftPid = new PIDController(0.03, 0, 0.005);

        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "LiftMotor");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ServoImplEx armServo = hardwareMap.get(ServoImplEx.class, "LiftArmServo");
        armServo.setPwmRange(new PwmControl.PwmRange(Arm.MIN_TICK, Arm.MAX_TICK));

        ServoImplEx grabberServo = hardwareMap.get(ServoImplEx.class, "LiftGrabServo");
        grabberServo.setPwmRange(new PwmControl.PwmRange(Grabber.MIN_TICK, Grabber.MAX_TICK));

        waitForStart();
        while(opModeIsActive()) {
            input.update();
            // control to stop timer (should manually be pressed to signal end of servo transition)
            if(input.getGamepadTracker1().isFirstFrameLeftBumper())
                lastTransitionDuration = transitionTime.seconds();

            // note: I chain together all grabber and arm inputs to only allow one servo movement at a time (for timer)
            if(input.getGamepadTracker1().isFirstFrameA()) {
                armServo.setPosition(Arm.TRANSFER_POS);
                transitionTime.reset();
            }
            else if(input.getGamepadTracker1().isFirstFrameB()) {
                armServo.setPosition(Arm.SPECIMEN_HANG_POS);
                transitionTime.reset();
            }
            else if(input.getGamepadTracker1().isFirstFrameY()) {
                armServo.setPosition(Arm.BASKET_SAFETY_POS);
                transitionTime.reset();
            }
            else if(input.getGamepadTracker1().isFirstFrameX()) {
                armServo.setPosition(Arm.BASKET_DROP_POS);
                transitionTime.reset();
            }
            else if(input.getGamepadTracker1().isFirstFrameDpadLeft()) {
                grabberServo.setPosition(Grabber.OPEN_POS);
                transitionTime.reset();
            }
            else if(input.getGamepadTracker1().isFirstFrameDpadRight()) {
                grabberServo.setPosition(Grabber.CLOSE_POS);
                transitionTime.reset();
            }


            Subsystem.setMotorPower(motor, -gamepad1.left_stick_y);

            telemetry.addData("current elapsed time", transitionTime.seconds());
            telemetry.addData("last transition time", lastTransitionDuration);
            telemetry.addData("", "");

            telemetry.addData("arm servo current pwm", armServo.getPosition());
            telemetry.addData("grabber servo current pwm", grabberServo.getPosition());
            telemetry.addData("motor current encoder: ", motor.getCurrentPosition());
            telemetry.addData("", "");

            telemetry.addData("controls below", "");
            telemetry.addData("left bumper", "stop timer");
            telemetry.addData("  lift", "");
            telemetry.addData("  left stick y", "moves lift up and down");
            telemetry.addData("arm", "");
            telemetry.addData("  a", "move arm to transfer");
            telemetry.addData("  b", "move arm to specimen hang");
            telemetry.addData("  y", "move arm to basket safety");
            telemetry.addData("  x", "move arm to basket drop");
            telemetry.addData("grabber", "");
            telemetry.addData("  dpad left", "open grabber");
            telemetry.addData("  dpad right", "close grabber");

            telemetry.update();


        }
    }
}
