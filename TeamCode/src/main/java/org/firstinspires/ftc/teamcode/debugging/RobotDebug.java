package org.firstinspires.ftc.teamcode.debugging;

import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;

public interface RobotDebug {
    void init_debug();
    void start_debug();
    void loop_debug() throws IllegalMessageTypeException;
    void sendMotionProfileData();

    void setUsingComputer(boolean usingComputer);
    boolean isUsingComputer();

    Pose2d getFieldPosition();
}
