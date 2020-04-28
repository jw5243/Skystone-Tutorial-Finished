package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.main.Robot;

public class MecanumRunnableSLQ implements Runnable {
    private static final int MAX_ITERATIONS = 5;
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private volatile boolean readyToUpdate;
    private boolean stop;

    private MecanumDriveSLQ slqDrivetrain;
    private double policyLag;

    private SimpleMatrix desiredState;

    public MecanumRunnableSLQ() {
        setTimeProfiler(new TimeProfiler(false));
        setPolicyTimeProfiler(new TimeProfiler(false));
        setReadyToUpdate(false);
        setStop(false);
        setPolicyLag(0d);
    }

    public MecanumDriveSLQ slq() {
        return slq(getDesiredState());
    }

    public MecanumDriveSLQ slq(SimpleMatrix desiredState) {
        MecanumDriveSLQ slq = new MecanumDriveSLQ(new MecanumDriveMPC(Robot.getDriveModel()));
        if(getDesiredState() == null) {
            setDesiredState(Robot.getInitialState());
        }

        slq.initialIteration(Robot.getState(), desiredState);
        for(int i = 0; i < getMaxIterations(); i++) {
            slq.simulateIteration();
            slq.runSLQ();
        }

        slq.simulateIteration();
        return slq;
    }

    @Override
    public void run() {
        getTimeProfiler().start();
        while(!isStop()) {
            if(!isReadyToUpdate()) {
                getPolicyTimeProfiler().start();
                setSlqDrivetrain(slq(getDesiredState()));
                getTimeProfiler().update(true);
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    e.printStackTrace();
                }

                setReadyToUpdate(true);
            }

            try {
                Thread.sleep(1);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public void updateSLQ() {
        if(isReadyToUpdate() && getSlqDrivetrain() != null) {
            Robot.setMecanumDriveSLQ(getSlqDrivetrain());
            setPolicyLag(getPolicyTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
            setReadyToUpdate(false);
        }
    }

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }

    public TimeProfiler getPolicyTimeProfiler() {
        return policyTimeProfiler;
    }

    public void setPolicyTimeProfiler(TimeProfiler policyTimeProfiler) {
        this.policyTimeProfiler = policyTimeProfiler;
    }

    public boolean isReadyToUpdate() {
        return readyToUpdate;
    }

    public void setReadyToUpdate(boolean readyToUpdate) {
        this.readyToUpdate = readyToUpdate;
    }

    public boolean isStop() {
        return stop;
    }

    public void setStop(boolean stop) {
        this.stop = stop;
    }

    public MecanumDriveSLQ getSlqDrivetrain() {
        return slqDrivetrain;
    }

    public void setSlqDrivetrain(MecanumDriveSLQ slqDrivetrain) {
        this.slqDrivetrain = slqDrivetrain;
    }

    public double getPolicyLag() {
        return policyLag;
    }

    public void setPolicyLag(double policyLag) {
        this.policyLag = policyLag;
    }

    public static int getMaxIterations() {
        return MAX_ITERATIONS;
    }

    public SimpleMatrix getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SimpleMatrix desiredState) {
        this.desiredState = desiredState;
    }

    public void setDesiredState(Pose2d desiredPose) {
        this.desiredState = new SimpleMatrix(6, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, 0d, desiredPose.getTranslation().y() * 0.0254d,
                0d, desiredPose.getRotation().getRadians(), 0d
        });
    }
}
