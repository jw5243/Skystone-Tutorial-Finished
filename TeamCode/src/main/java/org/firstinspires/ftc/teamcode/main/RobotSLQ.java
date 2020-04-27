package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.debugging.ComputerDebugger;
import org.firstinspires.ftc.teamcode.debugging.IllegalMessageTypeException;
import org.firstinspires.ftc.teamcode.debugging.MessageOption;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveSLQ;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableSLQ;
import org.firstinspires.ftc.teamcode.lib.control.Obstacle;
import org.firstinspires.ftc.teamcode.lib.geometry.Circle2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Line2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;

import java.util.ArrayList;
import java.util.List;

public class RobotSLQ extends Robot {
    private static List<Pose2d> positions = new ArrayList<>();
    private static List<Obstacle> obstacles = new ArrayList<>();

    static {
        //positions.add(new Pose2d(120, 120, new Rotation2d(Math.toRadians(180d), false)));
        positions.add(new Pose2d(100d, 50d, new Rotation2d(Math.toRadians(-135d), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(112d, 40d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));
        positions.add(new Pose2d(108d, 46d, new Rotation2d(Math.toRadians(-135), false)));
        positions.add(new Pose2d(104d, 120d, new Rotation2d(Math.toRadians(0d), false)));

        //obstacles.add(new Obstacle(94d, 64d, 4d, 10d));
        //obstacles.add(new Obstacle(94d, 70d, 4d, 10d));
        //obstacles.add(new Obstacle(94d, 76d, 4d, 1000d));
        obstacles.add(new Obstacle(94d, 82d, 4d, 0d));
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

        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 0.75d) && positions.size() > 1) {
            positions.remove(0);
            getMecanumRunnableSLQ().setDesiredState(positions.get(0));
        }

        try {
            for(int i = 0; i < getMecanumDriveSLQ().getSimulatedStates().length - 1; i++) {
                ComputerDebugger.send(MessageOption.LINE.setSendValue(
                        new Line2d(new Translation2d(
                                getMecanumDriveSLQ().getSimulatedStates()[i].get(0) / 0.0254d,
                                getMecanumDriveSLQ().getSimulatedStates()[i].get(2) / 0.0254d
                        ), new Translation2d(
                                getMecanumDriveSLQ().getSimulatedStates()[i + 1].get(0) / 0.0254d,
                                getMecanumDriveSLQ().getSimulatedStates()[i + 1].get(2) / 0.0254d
                        ))
                ));
            }

            for(int j = 0; j < getObstacles().size(); j++) {
                ComputerDebugger.send(MessageOption.KEY_POINT.setSendValue(new Circle2d(
                        getObstacles().get(j).getLocation(), getObstacles().get(j).getObstacleRadius() / 0.0254d
                )));
            }
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }

    public static List<Obstacle> getObstacles() {
        return obstacles;
    }
}
