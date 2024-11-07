package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.Action;
import org.firstinspires.ftc.teamcode.robot.CollectingSystem;

public class CollectingSystemAuto extends CollectingSystem {

    public CollectingSystemAuto(BrainSTEMRobotAuto robot) {
        super(robot);
    }
    
    public Action extendAndCollectAction(int extendMotorTick) {
        return new Action() {
            @Override
            public boolean run() {
                return new SequentialAction(
                    robot.getExtension().extendAction(extendMotorTick),
                    robot.getCollector().hingeAction(Collector.HINGE_DOWN_TICK),
                    robot.getCollector().collectAction(),
                    robot.getCollector().hingeAction(Collector.HINGE_UP_TICK),
                    robot.getExtension().retractAction()
                );
            }
        };
    }
}
