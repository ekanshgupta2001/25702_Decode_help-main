package org.firstinspires.ftc.teamcode.opmodes.scrap;

public class RobotTimer {
    long startTime;
    long waitTime;
    public boolean started;
    public RobotTimer(long time) {

        waitTime = time;
        started = false;
    }
    public void start() {
        startTime = System.currentTimeMillis();
        started = true;
    }
    public boolean IsDone() {
        long elapsed = System.currentTimeMillis() - startTime;
        if (elapsed > waitTime) {
            return true;
        }
        return false;
    }
    public void reset() {
        started = false;
    }
}
