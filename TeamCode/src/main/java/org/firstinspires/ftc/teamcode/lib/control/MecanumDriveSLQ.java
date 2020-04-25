package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleBase;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumDriveModel;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;

import java.util.Arrays;

public class MecanumDriveSLQ {
    private MecanumDriveMPC mecanumDriveMPC;
    private SimpleMatrix[] simulatedStates;
    private SimpleMatrix[] simulatedInputs;
    private SimpleMatrix[] A;
    private SimpleMatrix[] B;
    private SimpleMatrix[] P;
    private SimpleMatrix[] p;
    private SimpleMatrix[] K;
    private SimpleMatrix[] l;

    private SimpleMatrix initialState;
    private SimpleMatrix desiredState;

    public MecanumDriveSLQ(MecanumDriveMPC mecanumDriveMPC) {
        setMecanumDriveMPC(mecanumDriveMPC);
    }

    public static void main(String... args) {
        MecanumDriveSLQ slq = new MecanumDriveSLQ(new MecanumDriveMPC(new MecanumDriveModel(
                0.001d, 18.14d, 0.315d, 0.315d * (0.1d * 0.1d + 0.032d * 0.032d) / 2d,
                0.315d * (3d * (0.1d * 0.1d + 0.032d * 0.032d) + 0.05d * 0.05d) / 12d, 0.5613d,
                0.1d / 2d, 7d * 0.0254d, 7d * 0.0254d, 6d * 0.0254d, 6d * 0.0254d,
                MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, null))));

        SimpleMatrix state = new SimpleMatrix(6, 1, true, new double[] {
                0d, 0d, 0d, 0d, 0d, 0d
        });

        SimpleMatrix desiredState = new SimpleMatrix(6, 1, true, new double[] {
                10d * 0.0254d, 0d, 10d * 0.0254d, 0d, Math.toRadians(90d), 0d
        });

        slq.initialIteration(state, desiredState);
        slq.simulateIteration();
        slq.runSLQ();
        slq.simulateIteration();
        slq.runSLQ();
        slq.simulateIteration();
        slq.runSLQ();
        slq.simulateIteration();
        slq.runSLQ();
        slq.simulateIteration();
        slq.runSLQ();
        slq.simulateIteration();
        slq.runSLQ();
        System.out.println("t\tx\tvx\ty\tvy\tpsi\tvpsi");
        for(int i = 0; i < MecanumDriveMPC.getHorizonStep() - 1; i++) {
            state = slq.getMecanumDriveMPC().getModel().stateTransitionMatrix(state, MecanumDriveMPC.getDt(), true).mult(state)
                    .plus(slq.getMecanumDriveMPC().getModel().inputTransitionMatrix(state, MecanumDriveMPC.getDt(), false).mult(
                            slq.getOptimalInput(i, state, 0.001d)));
            System.out.println((int)(1000d * i * MecanumDriveMPC.getDt()) / 1000d + "\t" + state.get(0) / 0.0254d + "\t" + state.get(1) / 0.0254d + "\t" +
                    state.get(2) / 0.0254d + "\t" + state.get(3) / 0.0254d + "\t" + state.get(4) * 180d / Math.PI + "\t" + state.get(5) * 180d / Math.PI);
        }
    }

    public void initialIteration(SimpleMatrix initialState, Pose2d desiredPose) {
        initialIteration(initialState, new SimpleMatrix(6, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, 0d, desiredPose.getTranslation().y() * 0.0254d,
                0d, desiredPose.getRotation().getRadians(), 0d
        }));
    }

    public void initialIteration(SimpleMatrix initialState, SimpleMatrix desiredState) {
        setInitialState(initialState);
        setDesiredState(desiredState);
        getMecanumDriveMPC().runLQR(desiredState);
        K = getMecanumDriveMPC().getK();
        l = new SimpleMatrix[MecanumDriveMPC.getHorizonStep()];
        for(int i = 0; i < l.length; i++) {
            l[i] = new SimpleMatrix(4, 1, true, new double[] {
                    0d, 0d, 0d, 0d
            });
        }
    }

    public void simulateIteration() {
        simulateIteration(getInitialState(), getDesiredState());
    }

    public void simulateIteration(Pose2d desiredPose) {
        simulateIteration(getInitialState(), desiredPose);
    }

    public void simulateIteration(SimpleMatrix desiredState) {
        simulateIteration(getInitialState(), desiredState);
    }

    public void simulateIteration(SimpleMatrix initialState, Pose2d desiredPose) {
        simulateIteration(initialState, new SimpleMatrix(6, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, 0d, desiredPose.getTranslation().y() * 0.0254d,
                0d, desiredPose.getRotation().getRadians(), 0d
        }));
    }

    public void simulateIteration(SimpleMatrix initialState, SimpleMatrix desiredState) {
        if(getSimulatedStates() == null) {
            setSimulatedStates(new SimpleMatrix[MecanumDriveMPC.getHorizonStep() + 1]);
            setSimulatedInputs(new SimpleMatrix[MecanumDriveMPC.getHorizonStep()]);
            getSimulatedStates()[0] = initialState;
            setA(new SimpleMatrix[MecanumDriveMPC.getHorizonStep()]);
            setB(new SimpleMatrix[MecanumDriveMPC.getHorizonStep()]);
            for(int i = 1; i <= MecanumDriveMPC.getHorizonStep(); i++) {
                getA()[i - 1] = getMecanumDriveMPC().getModel().stateTransitionMatrix(getSimulatedStates()[i - 1], MecanumDriveMPC.getDt(), true);
                getB()[i - 1] = getMecanumDriveMPC().getModel().inputTransitionMatrix(getSimulatedStates()[i - 1], MecanumDriveMPC.getDt(), false);
                getSimulatedInputs()[i - 1] = getMecanumDriveMPC().getOptimalInput(i - 1, getSimulatedStates()[i - 1], desiredState);
                getSimulatedStates()[i] = getA()[i - 1].mult(getSimulatedStates()[i - 1]).plus(getB()[i - 1].mult(getSimulatedInputs()[i - 1]));
            }
        } else {
            for(int i = 1; i <= MecanumDriveMPC.getHorizonStep(); i++) {
                getA()[i - 1] = getMecanumDriveMPC().getModel().stateTransitionMatrix(getSimulatedStates()[i - 1], MecanumDriveMPC.getDt(), true);
                getB()[i - 1] = getMecanumDriveMPC().getModel().inputTransitionMatrix(getSimulatedStates()[i - 1], MecanumDriveMPC.getDt(), false);
                getSimulatedInputs()[i - 1] = getOptimalInput(i - 1, getSimulatedStates()[i - 1], 0.001d);
                getSimulatedStates()[i] = getA()[i - 1].mult(getSimulatedStates()[i - 1]).plus(getB()[i - 1].mult(getSimulatedInputs()[i - 1]));
            }
        }
    }

    public void runSLQ() {
        P = new SimpleMatrix[MecanumDriveMPC.getHorizonStep()];
        p = new SimpleMatrix[MecanumDriveMPC.getHorizonStep()];
        P[P.length - 1] = MecanumDriveMPC.getTerminationCost();
        p[p.length - 1] = getLinearStateCost(MecanumDriveMPC.getHorizonStep(), MecanumDriveMPC.getTerminationCost());
        solveRiccatiEquations(MecanumDriveMPC.getHorizonStep() - 1);
    }

    public void solveRiccatiEquations(int timeStep) {
        if(timeStep < 1) {
            return;
        }

        SimpleMatrix A = getA()[timeStep];
        SimpleMatrix B = getB()[timeStep];
        SimpleMatrix oldK = getMecanumDriveMPC().getK()[timeStep - 1];

        SimpleMatrix Q = MecanumDriveMPC.getStateCost(timeStep);
        SimpleMatrix R = MecanumDriveMPC.getInputCost();
        SimpleMatrix H = R.plus(B.transpose().mult(P[timeStep].mult(B)));
        SimpleMatrix G = B.transpose().mult(P[timeStep]).mult(A);
        P[timeStep - 1] = Q.plus(A.transpose().mult(P[timeStep].mult(A))).plus(oldK.transpose().mult(H)
                .mult(oldK)).plus(oldK.transpose().mult(G)).plus(G.transpose().mult(oldK));
        K[timeStep - 1] = H.invert().mult(G).negative();

        SimpleMatrix q = getLinearStateCost(timeStep, Q);
        SimpleMatrix r = getLinearInputCost(timeStep, R);
        SimpleMatrix g = r.plus(B.transpose().mult(p[timeStep]));

        p[timeStep - 1] = q.plus(A.transpose().mult(p[timeStep])).plus(oldK.transpose().mult(H)
                .mult(l[timeStep]))./*plus(l[timeStep].transpose().mult(G)).*/plus(G.transpose().mult(l[timeStep]).scale(2));
        l[timeStep - 1] = H.invert().mult(g).negative();

        solveRiccatiEquations(--timeStep);
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, double alpha) {
        if(timeStep < getSimulatedInputs().length - 1) {
            return MecanumDriveMPC.limitInput(getSimulatedInputs()[timeStep].plus(l[timeStep].scale(alpha)).plus(K[timeStep].mult(state.minus(getSimulatedStates()[timeStep]))));
        }

        return new SimpleMatrix(4, 1, true, new double[] {
                0d, 0d, 0d, 0d
        });
    }

    public SimpleMatrix getLinearStateCost(int timeStep, SimpleMatrix cost) {
        return cost.plus(cost.transpose()).mult(getSimulatedStates()[timeStep]).scale(-1 / 2d);
        //return cost.transpose().mult(getSimulatedStates()[timeStep]);
    }

    public SimpleMatrix getLinearInputCost(int timeStep, SimpleMatrix cost) {
        return cost.plus(cost.transpose()).mult(getSimulatedInputs()[timeStep]).scale(-1 / 2d);
        //return cost.transpose().mult(getSimulatedInputs()[timeStep]);
    }

    public MecanumDriveMPC getMecanumDriveMPC() {
        return mecanumDriveMPC;
    }

    public void setMecanumDriveMPC(MecanumDriveMPC mecanumDriveMPC) {
        this.mecanumDriveMPC = mecanumDriveMPC;
    }

    public SimpleMatrix[] getSimulatedStates() {
        return simulatedStates;
    }

    public void setSimulatedStates(SimpleMatrix[] simulatedStates) {
        this.simulatedStates = simulatedStates;
    }

    public SimpleMatrix[] getA() {
        return A;
    }

    public void setA(SimpleMatrix[] a) {
        A = a;
    }

    public SimpleMatrix[] getB() {
        return B;
    }

    public void setB(SimpleMatrix[] b) {
        B = b;
    }

    public SimpleMatrix[] getSimulatedInputs() {
        return simulatedInputs;
    }

    public void setSimulatedInputs(SimpleMatrix[] simulatedInputs) {
        this.simulatedInputs = simulatedInputs;
    }

    public SimpleMatrix getInitialState() {
        return initialState;
    }

    public void setInitialState(SimpleMatrix initialState) {
        this.initialState = initialState;
    }

    public SimpleMatrix getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(SimpleMatrix desiredState) {
        this.desiredState = desiredState;
    }
}
