package org.firstinspires.ftc.teamcode.robotStates.collectingSystemStates;

import org.firstinspires.ftc.teamcode.robotStates.RobotStateTele;
import org.firstinspires.ftc.teamcode.tele.CollectingSystemTele;

public abstract class CollectSystemState extends RobotStateTele<CollectingSystemTele.StateType> {

    protected CollectingSystemTele collectingSystemTele;
    public CollectSystemState(CollectingSystemTele.StateType anEnum) {
        super(anEnum);
    }

    @Override
    public void setup(Object...args) {
        super.setup(args);
        collectingSystemTele = (CollectingSystemTele) args[2];
    }
}
