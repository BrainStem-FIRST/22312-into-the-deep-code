package org.firstinspires.ftc.teamcode.auto;

public class TimedAction extends Action {
    
    private ElapsedTime timer;

    public TimedAction() {
        timer = new ElapsedTime();
    }

    public double getTime() {
        return timer.seconds();
    }
}
