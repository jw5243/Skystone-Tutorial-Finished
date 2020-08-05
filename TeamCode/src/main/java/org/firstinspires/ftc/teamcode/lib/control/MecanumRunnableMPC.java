package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.main.Robot;

public class MecanumRunnableMPC implements Runnable {
    private static final int MAX_ITERATIONS = 5;
    private TimeProfiler timeProfiler;
    private TimeProfiler policyTimeProfiler;
    private volatile boolean readyToUpdate;
    private boolean stop;

    private MecanumDriveMPC slqDrivetrain;
    private double policyLag;

    private SimpleMatrix desiredState;

    private static SimpleMatrix stateCost;
    private static SimpleMatrix inputCost;

    public MecanumRunnableMPC() {
        setTimeProfiler(new TimeProfiler(false));
        setPolicyTimeProfiler(new TimeProfiler(false));
        setReadyToUpdate(false);
        setStop(false);
        setPolicyLag(0d);
    }

    public MecanumDriveMPC slq() {
        return slq(getDesiredState());
    }

    public MecanumDriveMPC slq(SimpleMatrix desiredState) {
        MecanumDriveMPC slq = new MecanumDriveMPC(new MecanumDriveILQR(Robot.getDriveModel()));
        if(getStateCost() != null) {
            MecanumDriveILQR.setIntermediaryStateCost(getStateCost());
        }

        if(getInputCost() != null) {
            MecanumDriveILQR.setInputCost(getInputCost());
        }

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
                    Thread.sleep(1);
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
            Robot.setMecanumDriveMPC(getSlqDrivetrain());
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

    public MecanumDriveMPC getSlqDrivetrain() {
        return slqDrivetrain;
    }

    public void setSlqDrivetrain(MecanumDriveMPC slqDrivetrain) {
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

    public static SimpleMatrix getStateCost() {
        return stateCost;
    }

    public static void setStateCost(SimpleMatrix stateCost) {
        MecanumRunnableMPC.stateCost = stateCost;
    }

    public static SimpleMatrix getInputCost() {
        return inputCost;
    }

    public static void setInputCost(SimpleMatrix inputCost) {
        MecanumRunnableMPC.inputCost = inputCost;
    }
}
