package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveSLQ;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableSLQ;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

import java.util.ArrayList;
import java.util.List;

public class RobotSLQ extends Robot {
    private List<Pose2d> positions = new ArrayList<>();

    {
        positions.add(new Pose2d(120, 120, new Rotation2d(Math.toRadians(180d), false)));
        /*positions.add(new Pose2d(100d, 50d, new Rotation2d(Math.toRadians(-135d), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(112d, 40d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(108d, 46d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));*/
    }

    @Override
    public void init_debug() {
        super.init_debug();
        setMecanumDriveMPC(new MecanumDriveMPC(getDriveModel()));
        setMecanumDriveSLQ(new MecanumDriveSLQ(getMecanumDriveMPC()));

        getMecanumDriveSLQ().initialIteration(getState(), positions.get(0));
        for(int i = 0; i < MecanumRunnableSLQ.getMaxIterations(); i++) {
            getMecanumDriveSLQ().simulateIteration(getState(), positions.get(0));
            getMecanumDriveSLQ().runSLQ();
        }

        setMecanumRunnableSLQ(new MecanumRunnableSLQ());
        getMecanumRunnableSLQ().setDesiredState(positions.get(0));
        new Thread(getMecanumRunnableSLQ()).start();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        getMecanumRunnableSLQ().updateSLQ();
        setInput(getMecanumDriveSLQ().getOptimalInput((int)((getMecanumRunnableSLQ().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) +
                getMecanumRunnableSLQ().getPolicyLag()) / MecanumDriveMPC.getDt()), getState(), 0.001d));

        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 0.5d) && positions.size() > 1) {
            positions.remove(0);
            getMecanumRunnableSLQ().setDesiredState(positions.get(0));
            getMecanumRunnableSLQ().slq();
        }
    }
}
