package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.AllianceColor;
import org.firstinspires.ftc.teamcode.robot.BlockColorSensor;
import org.firstinspires.ftc.teamcode.robot.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;
import org.firstinspires.ftc.teamcode.robot.Collector;
import org.firstinspires.ftc.teamcode.robot.Extension;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "ExtensionTele")
public class ExtensionTele extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        BrainSTEMRobot robot = new BrainSTEMRobot(hardwareMap, telemetry, this, AllianceColor.BLUE);

        telemetry.addData("Opmode Status :", "Init");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()) {
            if(gamepad1.a) {
                if (robot.getExtension().getStateManager().getActiveStateType() == Extension.StateType.IN)
                    robot.getExtension().getStateManager().tryEnterState(Extension.StateType.FINDING_BLOCK);
                else
                    robot.getExtension().getStateManager().tryEnterState(Extension.StateType.RETRACTING);
            }
            if (gamepad1.y) {
                robot.getCollector().getStateManager().tryEnterState(Collector.StateType.HINGE_DOWN);
            }
            else if(gamepad1.b) {
                robot.getCollector().getStateManager().tryEnterState(Collector.StateType.HINGE_UP);
            }

            robot.update(0.016);
            /*
            BlockColorSensor colorSensor = robot.getCollector().getColorSensor();
            telemetry.addData("color sensor red, green, blue", colorSensor.red() + " " + colorSensor.green() + " " + colorSensor.blue());

            robot.telemetry.addData("has red", robot.getCollector().getColorSensor().hasColor(BlockColorSensor.BlockColor.RED));
            robot.telemetry.addData("has yellow", robot.getCollector().getColorSensor().hasColor(BlockColorSensor.BlockColor.YELLOW));
            robot.telemetry.addData("has blue", robot.getCollector().getColorSensor().hasColor(BlockColorSensor.BlockColor.BLUE));
            robot.telemetry.addData("color sensor block color", robot.getCollector().getColorSensor().getBlockColor());
            */
            telemetry.addData("a", gamepad1.a);
            telemetry.addData("b",gamepad1.b);
            telemetry.addData("y", gamepad1.y);
            telemetry.addData("extension state", robot.getExtension().getStateManager().getActiveStateType());
            telemetry.addData("extension position", robot.getExtension().getExtensionMotor().getCurrentPosition());
            telemetry.addData("collector state", robot.getCollector().getStateManager().getActiveStateType());
            telemetry.update();
        }
    }
}
