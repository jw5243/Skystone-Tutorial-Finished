package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableLQR;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

public class AutonomousRobot extends Robot {
    private Pose2d finalPosition = new Pose2d(10 * 12d, 10 * 12d, new Rotation2d(Math.toRadians(180d), false));

    @Override
    public void init_debug() {
        super.init_debug();
        setMecanumDriveMPC(new MecanumDriveMPC(true));
        getMecanumDriveMPC().model = getDriveModel();
        getMecanumDriveMPC().runLQR(finalPosition);
        setMecanumDriveRunnableLQR(new MecanumRunnableLQR(finalPosition));
        new Thread(getMecanumDriveRunnableLQR()).start();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        getMecanumDriveRunnableLQR().updateMPC();
        setInput(getMecanumDriveMPC().getOptimalInput((int)((getMecanumDriveRunnableLQR().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) +
                getMecanumDriveRunnableLQR().getPolicyLag()) / MecanumDriveMPC.dt), getState(), getInput(), false));
    }
}
