package org.firstinspires.ftc.teamcode.main;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveILQR;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableMPC;
import org.firstinspires.ftc.teamcode.lib.control.Obstacle;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.lib.util.TimeUtil;

import java.util.ArrayList;
import java.util.List;

public class RobotGAMPC extends Robot {
    private static List<Pose2d> positions   = new ArrayList<>();
    private static List<Obstacle> obstacles = new ArrayList<>();

    private double runtime = 0d;
    private boolean isDone = false;

    static {
        //GF Path
        /*positions.add(new Pose2d(38d, 34d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(38d, 144d - 11d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(38d, 10d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(38d, 144d - 24d, new Rotation2d(Math.toRadians(-90d), false)));
        positions.add(new Pose2d(46d, 40d, new Rotation2d(Math.toRadians(-45d), false)));
        positions.add(new Pose2d(40d, 144d - 24d, new Rotation2d(Math.toRadians(-180d), false)));
        positions.add(new Pose2d(36d, 144d - 40d, new Rotation2d(Math.toRadians(-90), false)));
        positions.add(new Pose2d(46d, 34d, new Rotation2d(Math.toRadians(-45d), false)));
        positions.add(new Pose2d(30d, 144d - 19d - 9d - 6d, new Rotation2d(Math.toRadians(-90), false)));
        positions.add(new Pose2d(46d, 26d, new Rotation2d(Math.toRadians(-45d), false)));
        positions.add(new Pose2d(30d, 144d - 19d - 9d - 6d, new Rotation2d(Math.toRadians(-90), false)));
        positions.add(new Pose2d(46d, 14d, new Rotation2d(Math.toRadians(-45d), false)));
        positions.add(new Pose2d(30d, 144d - 19d - 9d - 6d, new Rotation2d(Math.toRadians(-90), false)));
        positions.add(new Pose2d(110d, 72d, new Rotation2d(Math.toRadians(-90d), false)));

        obstacles.add(new Obstacle(144d - 99d, 61d, 3d, 300d));
        obstacles.add(new Obstacle(144d - 99d, 84d, 3d, 300d));
        obstacles.add(new Obstacle(144d - (144d - 9d), 90d, 10.5d, 300d));*/

        ///////////////////////////////////////////////////////////////////////////////////////////

        positions.add(new Pose2d(120, 120, new Rotation2d(Math.toRadians(90d), false)));
    }

    @Override
    public void init_debug() {
        super.init_debug();
        setMecanumDriveILQR(new MecanumDriveILQR(getDriveModel()));
        setMecanumDriveMPC(new MecanumDriveMPC(getMecanumDriveILQR()));

        getMecanumDriveMPC().initialIteration(getState(), positions.get(0));
        for(int i = 0; i < MecanumRunnableMPC.getMaxIterations(); i++) {
            getMecanumDriveMPC().simulateIteration(getState(), positions.get(0));
            getMecanumDriveMPC().runMPCIteration();
        }

        setMecanumRunnableMPC(new MecanumRunnableMPC());
        getMecanumRunnableMPC().setDesiredState(positions.get(0));
        new Thread(getMecanumRunnableMPC()).start();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();

        getMecanumRunnableMPC().updateSLQ();
        setInput(getMecanumDriveMPC().getOptimalInput((int)((getMecanumRunnableMPC().getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, false) +
                getMecanumRunnableMPC().getPolicyLag()) / MecanumDriveILQR.getDt()), getState(), 0.001d));

        if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 2.5d) && positions.size() > 1) {
            positions.remove(0);
            getMecanumRunnableMPC().setDesiredState(positions.get(0));
        } else if(getFieldPosition().getTranslation().epsilonEquals(positions.get(0).getTranslation(), 1d) && positions.size() == 1) {
            stopTimer();
            setDone(true);
            setRuntime(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS));
            setInput(new SimpleMatrix(4, 1, true, new double[] {
                    0, 0, 0, 0
            }));
        }

        /*try {
            for(int i = 0; i < getMecanumDriveMPC().getSimulatedStates().length - 1; i++) {
                if(Double.isFinite(getMecanumDriveMPC().getSimulatedStates()[i].get(0)) &&
                        Double.isFinite(getMecanumDriveMPC().getSimulatedStates()[i].get(2)) &&
                        Double.isFinite(getMecanumDriveMPC().getSimulatedStates()[i + 1].get(0)) &&
                        Double.isFinite(getMecanumDriveMPC().getSimulatedStates()[i + 1].get(2))) {
                    ComputerDebugger.send(MessageOption.LINE.setSendValue(
                            new Line2d(new Translation2d(
                                    getMecanumDriveMPC().getSimulatedStates()[i].get(0) / 0.0254d,
                                    getMecanumDriveMPC().getSimulatedStates()[i].get(2) / 0.0254d
                            ), new Translation2d(
                                    getMecanumDriveMPC().getSimulatedStates()[i + 1].get(0) / 0.0254d,
                                    getMecanumDriveMPC().getSimulatedStates()[i + 1].get(2) / 0.0254d
                            ))
                    ));
                }
            }

            for(int j = 0; j < getObstacles().size(); j++) {
                ComputerDebugger.send(MessageOption.KEY_POINT.setSendValue(new Circle2d(
                        getObstacles().get(j).getLocation(), getObstacles().get(j).getObstacleRadius() / 0.0254d
                )));
            }
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }*/
    }

    public static List<Obstacle> getObstacles() {
        return obstacles;
    }

    public double getRuntime() {
        return runtime;
    }

    public void setRuntime(double runtime) {
        this.runtime = runtime;
    }

    public boolean isDone() {
        return isDone;
    }

    public void setDone(boolean done) {
        isDone = done;
    }

    public static List<Pose2d> getPositions() {
        return positions;
    }
}
