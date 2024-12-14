package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImpl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.driveTrain.PinpointDrive;
import org.firstinspires.ftc.teamcode.robot.Arm;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;
import org.firstinspires.ftc.teamcode.robot.Grabber;
import org.firstinspires.ftc.teamcode.robot.Hinge;
import org.firstinspires.ftc.teamcode.robot.Lift;
import org.firstinspires.ftc.teamcode.robot.Subsystem;
import org.firstinspires.ftc.teamcode.util.Input;
import org.firstinspires.ftc.teamcode.util.MotorCurrentTracker;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "BasicControlsTele")
public class BasicControlsTele extends LinearOpMode {
    private Input input;
    private PinpointDrive driveTrain;

    private DcMotorEx liftMotor;
    private ServoImplEx armServo, grabberServo;

    private DcMotorEx extensionMotor, spindleMotor;
    private ServoImplEx hingeServo;
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d BEGIN_POSE = new Pose2d(-24, -7.5, Math.toRadians(90));

        input = new Input(gamepad1, gamepad2);
        driveTrain = new PinpointDrive(hardwareMap, BEGIN_POSE);

        // setting up lifting system
        liftMotor = hardwareMap.get(DcMotorEx.class, "LiftMotor");
        liftMotor.setDirection(Lift.LIFT_DIRECTION);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armServo = hardwareMap.get(ServoImplEx.class, "LiftArmServo");
        armServo.setPwmRange(new PwmControl.PwmRange(Arm.MIN_TICK, Arm.MAX_TICK));

        grabberServo = hardwareMap.get(ServoImplEx.class, "LiftGrabServo");
        grabberServo.setPwmRange(new PwmControl.PwmRange(Grabber.MIN_TICK, Grabber.MAX_TICK));

        // setting up collecting system
        extensionMotor = hardwareMap.get(DcMotorEx.class, "ExtensionMotor");
        extensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        spindleMotor = hardwareMap.get(DcMotorEx.class, "CollectSpindleMotor");
        hingeServo = hardwareMap.get(ServoImplEx.class, "CollectHingeServo");
        hingeServo.setPwmRange(new PwmControl.PwmRange(Hinge.HINGE_DOWN_TICK, Hinge.HINGE_UP_TICK));

        waitForStart();

        while (opModeIsActive()) {
            input.update();

            listenForDriveTrainInput();
            listenForBasicLiftSystemInput();
            listenForBasicCollectSystemInput();

            driveTrain.updatePoseEstimate();

            telemetry.addData("pose", "");
            telemetry.addData("  x", driveTrain.pose.position.x);
            telemetry.addData("  y", driveTrain.pose.position.y);
            telemetry.addData("  heading", driveTrain.pose.heading.toDouble());

            telemetry.addData("", "");
            telemetry.addData("subsystem data", "");
            telemetry.addData("  lift encoder", liftMotor.getCurrentPosition());
            telemetry.addData("  arm pwm", armServo.getPosition());
            telemetry.addData("  grabber pwm", grabberServo.getPosition());
            telemetry.addData("  extension encoder", extensionMotor.getCurrentPosition());
            telemetry.addData("  hinge pwm", hingeServo.getPosition());

            telemetry.addData("", "");
            telemetry.addData("controls (all on gamepad1)", "");
            telemetry.addData("  joysticks", "drivetrain");
            telemetry.addData("  dpad up/down", "lift");
            telemetry.addData("  dpad left/right", "grabber");
            telemetry.addData("  buttons", "arm");
            telemetry.addData("  bumpers", "extension");
            telemetry.addData("  left trigger", "spit collector");
            telemetry.addData("  right trigger", "toggle hinge and collect when hinged down");
            
            telemetry.update();
        }
    }
    private void listenForDriveTrainInput() {
        final double STRAFE_Y_AMP = 0.6;
        final double TURN_AMP = 0.8;

        int strafeDirY = input.getGamepadTracker1().isDpadRightPressed() ? 1 : input.getGamepadTracker1().isDpadLeftPressed() ? -1 : 0;

        if (strafeDirY != 0)
            driveTrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0, -strafeDirY * STRAFE_Y_AMP),
                    0
            ));
        else
            driveTrain.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x),
                    -gamepad1.right_stick_x * TURN_AMP
            ));
    }
    private void listenForBasicLiftSystemInput() {
        // arm input
        if(input.getGamepadTracker1().isFirstFrameA()) {
            armServo.setPosition(Arm.TRANSFER_POS);
        }
        else if(input.getGamepadTracker1().isFirstFrameB()) {
            armServo.setPosition(Arm.BASKET_DROP_POS);
        }
        else if(input.getGamepadTracker1().isFirstFrameY()) {
            armServo.setPosition(Arm.BASKET_SAFETY_POS);
        }
        else if(input.getGamepadTracker1().isFirstFrameX()) {
            armServo.setPosition(Arm.SPECIMEN_HANG_POS);
        }

        // grabber input
        if(input.getGamepadTracker1().isFirstFrameDpadLeft()) {
            grabberServo.setPosition(Grabber.OPEN_POS);
        }
        else if(input.getGamepadTracker1().isFirstFrameDpadRight()) {
            grabberServo.setPosition(Grabber.CLOSE_POS);
        }

        // lift input
        if(input.getGamepadTracker1().isFirstFrameRightBumper())
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        else if(input.getGamepadTracker1().isDpadUpPressed())
            Subsystem.setMotorPower(liftMotor, 0.5);
        else if(input.getGamepadTracker1().isDpadDownPressed())
            Subsystem.setMotorPower(liftMotor, -0.5);
        else
            Subsystem.setMotorPower(liftMotor, 0.1);


    }
    private void listenForBasicCollectSystemInput() {
        // extension
        if (input.getGamepadTracker1().isRightBumperPressed())
            Subsystem.setMotorPower(extensionMotor, Extension.SEARCH_POWER);
        else if (input.getGamepadTracker1().isLeftBumperPressed())
            Subsystem.setMotorPower(extensionMotor, -Extension.SEARCH_POWER);
        else
            Subsystem.setMotorPower(extensionMotor, 0);

        // collector motor
        if(input.getGamepadTracker1().isLeftTriggerPressed())
            Subsystem.setMotorPower(spindleMotor, Collector.SPIT_POWER);
        else if (hingeServo.getPosition() == Hinge.HINGE_DOWN_POSITION)
            Subsystem.setMotorPower(spindleMotor, Collector.TELE_COLLECT_POWER);
        else
            Subsystem.setMotorPower(spindleMotor, 0);

        // hinge
        if (input.getGamepadTracker1().isRightTriggerPressed())
            if(hingeServo.getPosition() == Hinge.HINGE_UP_POSITION)
                hingeServo.setPosition(Hinge.HINGE_DOWN_POSITION);
            else if(hingeServo.getPosition() == Hinge.HINGE_DOWN_POSITION)
                hingeServo.setPosition(Hinge.HINGE_UP_POSITION);

    }
    private void listenForDriveTrainInputOld() {
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
            driveTrain.setMotorPowers((addValue + rightStickX), (subtractValue - rightStickX), (subtractValue + rightStickX), (addValue - rightStickX));
        } else {
            driveTrain.stop();
        }
    }
}
