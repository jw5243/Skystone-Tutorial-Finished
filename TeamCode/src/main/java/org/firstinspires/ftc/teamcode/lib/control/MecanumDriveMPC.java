package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rectangle;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumDriveModel;
import org.firstinspires.ftc.teamcode.lib.util.MatrixUtil;
import org.firstinspires.ftc.teamcode.main.Robot;

import java.util.ArrayList;
import java.util.List;

public class MecanumDriveMPC {
    private static final int    HORIZON_STEP = 1000;
    private static final double dt           = 0.001d;

    private static final SimpleMatrix TERMINATION_COST = new SimpleMatrix(6, 6, false, new double[] {
            100, 0, 0, 0, 0, 0,
            0, 100, 0, 0, 0, 0,
            0, 0, 100, 0, 0, 0,
            0, 0, 0, 100, 0, 0,
            0, 0, 0, 0, 100, 0,
            0, 0, 0, 0, 0, 100
    });

    private static final SimpleMatrix INTERMEDIARY_STATE_COST = new SimpleMatrix(6, 6, false, new double[] {
            1E-19, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 1E-19, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 1E-20, 0,
            0, 0, 0, 0, 0, 0,
    });

    private static final SimpleMatrix INPUT_COST = new SimpleMatrix(4, 4, false, new double[] {
            1E-20, 0, 0, 0,
            0, 1E-20, 0, 0,
            0, 0, 1E-20, 0,
            0, 0, 0, 1E-20
    });

    private static final SimpleMatrix INPUT_CHANGE_COST = new SimpleMatrix(4, 4, false, new double[] {
            1E-20, 0, 0, 0,
            0, 1E-20, 0, 0,
            0, 0, 1E-20, 0,
            0, 0, 0, 1E-20
    });

    private MecanumDriveModel model;
    private SimpleMatrix[] P;
    private SimpleMatrix[] K;
    private SimpleMatrix state;
    private SimpleMatrix finalState;
    private SimpleMatrix lastInput;
    private int timeStep;

    public void initializeState() {
        setState(Robot.getState());
        setLastInput(Robot.getInput());
    }

    public void initializeState(Pose2d position) {
        setState(new SimpleMatrix(6, 1, false, new double[] {
                position.getTranslation().x() * 0.0254d, 0, position.getTranslation().y() * 0.0254d, 0, position.getRotation().getRadians(), 0
        }));

        setLastInput(new SimpleMatrix(4, 1, false, new double[] {
                0, 0, 0, 0
        }));
    }

    public void initializeState(SimpleMatrix state) {
        setState(state);
        setLastInput(new SimpleMatrix(4, 1, false, new double[] {
                0, 0, 0, 0
        }));
    }

    public MecanumDriveMPC(boolean selfInitialize) {
        if(selfInitialize) {
            initializeState();
        }
    }

    public static void main(String... args) {
        MecanumDriveModel model = new MecanumDriveModel(3000, 0.001, 15.75d, 0.315d, 0.315d * (0.1 * 0.1 + 0.032 * 0.032) / 2,
                0.315d * (3 * (0.1 * 0.1 + 0.032 * 0.032) + 0.05 * 0.05) / 12, 0.5613d,
                0.1d / 2, 13.7d, 2d, 12d, 0.187d, 9.2d,
                435 * 2 * Math.PI / 60d, 0.25d, 0.6d,
                7d * 0.0254, 7d * 0.0254, 7d * 0.0254, 7d * 0.0254);

        MecanumDriveMPC lqr = new MecanumDriveMPC(false);
        lqr.model = model;
        lqr.initializeState(new Pose2d(0d, 0d, new Rotation2d(0d, true)));
        lqr.runLQR(new Pose2d(100d, 100d, new Rotation2d(Math.toRadians(90d), true)));
        for(int i = 0; i < HORIZON_STEP; i++) {
            lqr.simulateSingleTimeStep();
            System.out.print(lqr.state);
            System.out.println(lqr.lastInput);
        }
    }

    public List<Translation2d> disallowedPositions(List<Rectangle> obstacles, SimpleMatrix A, SimpleMatrix B, SimpleMatrix state) {
        List<Translation2d> disallowedPositions = new ArrayList<>();
        SimpleMatrix C = new SimpleMatrix(2, 6, true, new double[] {
                1, 0, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0
        });

        SimpleMatrix tildaA = A.plus(B.mult(K[0]));
        SimpleMatrix tildaB = B.mult(K[0]).negative();
        SimpleMatrix F = MatrixUtil.expAt(tildaA, getDt());
        SimpleMatrix G = tildaA.invert().mult(F.minus(SimpleMatrix.identity(6))).mult(tildaB);
        for(int i = 0; i < getHorizonStep(); i++) {
            for(Rectangle obstacle : obstacles) {
                for(SimpleMatrix matrix : obstacle.minkowskiSum(C.mult(F).mult(state).negative())) {
                    System.out.println(C.mult(G));
                    SimpleMatrix disallowedPosition = C.mult(G).invert().mult(matrix);
                    disallowedPositions.add(new Translation2d(disallowedPosition.get(0) / 0.0254d, disallowedPosition.get(1) / 0.0254d));
                }
            }

            if(i + 1 < getHorizonStep()) {
                tildaA = A.plus(B.mult(K[i + 1]));
                tildaB = B.mult(K[i + 1]).negative();
                F = F.mult(MatrixUtil.expAt(tildaA, getDt()));
                G = G.mult(tildaA.invert().mult(F.minus(SimpleMatrix.identity(6))).mult(tildaB));
            }
        }

        return disallowedPositions;
    }

    public List<Translation2d> disallowedPositions(List<Rectangle> obstacles, SimpleMatrix state) {
        return disallowedPositions(obstacles, getModel().stateTransitionMatrix(state, getDt(), true),
                getModel().inputTransitionMatrix(state, getDt(), false), state);
    }

    /**
     * Run-to-position
     *
     * @param finalPosition
     */
    public void runLQR(Pose2d finalPosition) {
        SimpleMatrix finalState = new SimpleMatrix(6, 1, false, new double[] {
                finalPosition.getTranslation().x() * 0.0254d, 0, finalPosition.getTranslation().y() * 0.0254d, 0, finalPosition.getRotation().getRadians(), 0
        });

        runLQR(getState(), finalState, getLastInput());
    }

    public void runLQR(SimpleMatrix finalState) {
        runLQR(getState(), new SimpleMatrix(6, 1, false, new double[] {
                finalState.get(0) * 0.0254d, finalState.get(1) * 0.0254d, finalState.get(2) * 0.0254d,
                finalState.get(3) * 0.0254d, finalState.get(4), finalState.get(5)
        }), getLastInput());
    }

    public void runLQR(SimpleMatrix initialState, SimpleMatrix finalState, SimpleMatrix lastInput) {
        setTimeStep(0);
        setState(initialState);
        setFinalState(finalState);
        setLastInput(lastInput);
        P = new SimpleMatrix[HORIZON_STEP];
        K = new SimpleMatrix[HORIZON_STEP - 1];
        P[P.length - 1] = getStateCost(HORIZON_STEP);

        SimpleMatrix A = model.stateTransitionMatrix(state, true);
        SimpleMatrix B = model.inputTransitionMatrix(state, false);

        solveRiccatiEquation(HORIZON_STEP - 1, A, B);
    }

    public SimpleMatrix simulate(double dt) {
        state = model.simulateDynamics(state, getOptimalInput(timeStep, state, lastInput), dt);
        timeStep += (int) (dt / MecanumDriveMPC.dt);
        return state;
    }

    public SimpleMatrix simulateSingleTimeStep() {
        state = model.simulateDynamics(state, getOptimalInput(timeStep++, state, lastInput));
        return state;
    }

    public SimpleMatrix simulateSingleTimeStep(SimpleMatrix state) {
        return model.simulateDynamics(state, getOptimalInput(timeStep++, state, lastInput));
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, SimpleMatrix input) {
        if(timeStep < K.length) {
            lastInput = limitInput(K[timeStep].mult(state.minus(finalState)));
        } else {
            lastInput = new SimpleMatrix(4, 1);
        }

        return lastInput;
    }

    public void solveRiccatiEquation(int timeStep, SimpleMatrix A, SimpleMatrix B) {
        if(timeStep < 1) {
            return;
        }

        SimpleMatrix Q = getStateCost(timeStep);
        SimpleMatrix R = INPUT_COST;
        SimpleMatrix inverse = R.plus(B.transpose().mult(P[timeStep].mult(B))).pseudoInverse();
        P[timeStep - 1] = Q.plus(A.transpose().mult(P[timeStep].mult(A))).minus(A.transpose().mult(P[timeStep].mult(B.mult(inverse).mult(B.transpose().mult(P[timeStep].mult(A))))));
        K[timeStep - 1] = inverse.mult(B.transpose()).mult(P[timeStep]).mult(A).negative();
        solveRiccatiEquation(--timeStep, A, B);
    }

    public static SimpleMatrix limitInput(SimpleMatrix control) {
        return new SimpleMatrix(4, 1, false, new double[] {
                control.get(0) > 1d ? 1d : control.get(0) < -1d ? -1d : control.get(0),
                control.get(1) > 1d ? 1d : control.get(1) < -1d ? -1d : control.get(1),
                control.get(2) > 1d ? 1d : control.get(2) < -1d ? -1d : control.get(2),
                control.get(3) > 1d ? 1d : control.get(3) < -1d ? -1d : control.get(3)
        });
    }

    public static double calculateCostToGo(int timeStep, SimpleMatrix state, SimpleMatrix input) {
        return state.transpose().mult(getStateCost(timeStep)).mult(state).plus(input.transpose().mult(INPUT_COST).mult(input)).get(0, 0);
    }

    private static SimpleMatrix augmentState(SimpleMatrix state, SimpleMatrix input) {
        return new SimpleMatrix(10, 1, false, new double[] {
                state.get(0),
                state.get(1),
                state.get(2),
                state.get(3),
                state.get(4),
                state.get(5),
                input.get(0),
                input.get(1),
                input.get(2),
                input.get(3)
        });
    }

    private static SimpleMatrix getStateCost(int timeStep) {
        return timeStep >= HORIZON_STEP - 1 ? TERMINATION_COST : INTERMEDIARY_STATE_COST;
    }

    public static int getHorizonStep() {
        return HORIZON_STEP;
    }

    public static double getDt() {
        return dt;
    }

    public static SimpleMatrix getTerminationCost() {
        return TERMINATION_COST;
    }

    public static SimpleMatrix getIntermediaryStateCost() {
        return INTERMEDIARY_STATE_COST;
    }

    public static SimpleMatrix getInputCost() {
        return INPUT_COST;
    }

    public static SimpleMatrix getInputChangeCost() {
        return INPUT_CHANGE_COST;
    }

    public MecanumDriveModel getModel() {
        return model;
    }

    public void setModel(MecanumDriveModel model) {
        this.model = model;
    }

    public SimpleMatrix[] getP() {
        return P;
    }

    public void setP(SimpleMatrix[] p) {
        P = p;
    }

    public SimpleMatrix[] getK() {
        return K;
    }

    public void setK(SimpleMatrix[] k) {
        K = k;
    }

    public SimpleMatrix getState() {
        return state;
    }

    public void setState(SimpleMatrix state) {
        this.state = state;
    }

    public SimpleMatrix getFinalState() {
        return finalState;
    }

    public void setFinalState(SimpleMatrix finalState) {
        this.finalState = finalState;
    }

    public SimpleMatrix getLastInput() {
        return lastInput;
    }

    public void setLastInput(SimpleMatrix lastInput) {
        this.lastInput = lastInput;
    }

    public int getTimeStep() {
        return timeStep;
    }

    public void setTimeStep(int timeStep) {
        this.timeStep = timeStep;
    }
}
