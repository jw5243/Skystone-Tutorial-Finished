package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.debugging.RobotDebug;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public abstract class Robot implements RobotDebug {
    private static boolean isUsingComputer;

    private TimeProfiler timeProfiler;
    private double dt;

    @Override
    public void init_debug() {
        setTimeProfiler(new TimeProfiler(false));
        setDt(0d);
    }

    @Override
    public void start_debug() {
        getTimeProfiler().start();
    }

    @Override
    public void loop_debug() {
        setDt(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
    }

    @Override
    public void sendMotionProfileData() {

    }

    @Override
    public Pose2d getFieldPosition() {
        return new Pose2d();
    }

    public static boolean isUsingComputer() {
        return isUsingComputer;
    }

    public static void setUsingComputer(boolean isUsingComputer) {
        Robot.isUsingComputer = isUsingComputer;
    }

    public void setDt(double dt) {
        this.dt = dt;
    }

    public double getDt() {
        return dt;
    }

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }
}
