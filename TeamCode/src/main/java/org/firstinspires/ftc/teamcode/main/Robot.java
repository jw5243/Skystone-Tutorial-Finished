package org.firstinspires.ftc.teamcode.main;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.debugging.RobotDebug;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public abstract class Robot implements RobotDebug {
    private static final SimpleMatrix INITIAL_STATE = new SimpleMatrix(6, 1, false, new double[] {
            9d, 0d, 9d, 0d, 0d, 0d
    });

    private static boolean isUsingComputer;

    private SimpleMatrix state;

    private TimeProfiler timeProfiler;
    private double dt;

    @Override
    public void init_debug() {
        setState(getInitialState());
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
        return new Pose2d(9d, 9d, new Rotation2d(0d, false));
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

    public static SimpleMatrix getInitialState() {
        return INITIAL_STATE;
    }

    public SimpleMatrix getState() {
        return state;
    }

    public void setState(SimpleMatrix state) {
        this.state = state;
    }
}
