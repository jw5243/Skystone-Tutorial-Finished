package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.debugging.IllegalMessageTypeException;
import org.firstinspires.ftc.teamcode.debugging.RobotDebug;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;

public abstract class Robot implements RobotDebug {
    @Override
    public void init_debug() {

    }

    @Override
    public void start_debug() {

    }

    @Override
    public void loop_debug() throws IllegalMessageTypeException {

    }

    @Override
    public void sendMotionProfileData() {

    }

    @Override
    public void setUsingComputer(boolean usingComputer) {

    }

    @Override
    public boolean isUsingComputer() {
        return false;
    }

    @Override
    public Pose2d getFieldPosition() {
        return null;
    }
}
