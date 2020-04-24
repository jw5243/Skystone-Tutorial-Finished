package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableLQR;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class AutonomousRobot extends Robot {
    private List<Pose2d> positions = new ArrayList<>();

    {
        positions.add(new Pose2d(100d, 50d, new Rotation2d(Math.toRadians(-135d), false)));
        //positions.add(new Pose2d(116d, 120d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(112d, 40d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(108d, 46d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));
    }

    @Override
    public void init_debug() {
        super.init_debug();
        setMecanumDriveMPC(new MecanumDriveMPC());
        getMecanumDriveMPC().setModel(getDriveModel());
        getMecanumDriveMPC().runLQR(getState());
        //Arrays.stream(getMecanumDriveMPC().getK()).forEach(matrix -> matrix.scale(1 / 0.0254d).print());
        setMecanumDriveRunnableLQR(new MecanumRunnableLQR());
        new Thread(getMecanumDriveRunnableLQR()).start();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        getMecanumDriveRunnableLQR().updateMPC();
        setInput(getMecanumDriveMPC().getOptimalInput((int)((getMecanumDriveRunnableLQR().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) +
                getMecanumDriveRunnableLQR().getPolicyLag()) / MecanumDriveMPC.getDt()), getState(), positions.get(0)));

        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 0.1d) && positions.size() > 1) {
            positions.remove(0);
            getMecanumDriveMPC().runLQR(getState());
        }
    }
}
