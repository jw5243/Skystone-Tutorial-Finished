package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.debugging.IllegalMessageTypeException;
import org.firstinspires.ftc.teamcode.debugging.RobotDebug;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;

public abstract class Robot implements RobotDebug {
    private static boolean isUsingComputer;

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
    public Pose2d getFieldPosition() {
        return null;
    }

    public static boolean isUsingComputer() {
        return isUsingComputer;
    }

    public static void setUsingComputer(boolean isUsingComputer) {
        Robot.isUsingComputer = isUsingComputer;
    }
}
