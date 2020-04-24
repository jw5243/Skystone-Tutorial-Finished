package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumDriveModel;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;

public class MecanumDriveMPC {
    private static final int    HORIZON_STEP = 1000;
    private static final double dt           = 0.01d;

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

    private MecanumDriveModel model;
    private SimpleMatrix[] P;
    private SimpleMatrix[] K;

    public static void main(String... args) {
        MecanumDriveModel model = new MecanumDriveModel(
                0.001d, 18.14d, 0.315d, 0.315d * (0.1d * 0.1d + 0.032d * 0.032d) / 2d,
                0.315d * (3d * (0.1d * 0.1d + 0.032d * 0.032d) + 0.05d * 0.05d) / 12d, 0.5613d,
                0.1d / 2d, 7d * 0.0254d, 7d * 0.0254d, 6d * 0.0254d, 6d * 0.0254d,
                MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, null));

        MecanumDriveMPC lqr = new MecanumDriveMPC();
        SimpleMatrix state = new SimpleMatrix(6, 1, true, new double[] {
                0d, 0d, 0d, 0d, 0d, 0d
        });

        lqr.model = model;
        lqr.runLQR(state);
        for(int i = 0; i < HORIZON_STEP; i++) {
            state = model.simulateDynamics(state, lqr.getOptimalInput(i, state, new Pose2d(10, 0, new Rotation2d(Math.toRadians(0d), true))));
            System.out.print(state);
        }
    }

    public void runLQR(SimpleMatrix initialState) {
        P = new SimpleMatrix[HORIZON_STEP];
        K = new SimpleMatrix[HORIZON_STEP - 1];
        P[P.length - 1] = getStateCost(HORIZON_STEP);

        SimpleMatrix A = model.stateTransitionMatrix(initialState, getDt(), true);
        SimpleMatrix B = model.inputTransitionMatrix(initialState, getDt(), false);

        solveRiccatiEquation(HORIZON_STEP - 1, A, B);
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, Pose2d desiredPose) {
        return getOptimalInput(timeStep, state, new SimpleMatrix(6, 1, true, new double[] {
                desiredPose.getTranslation().x() * 0.0254d, 0d, desiredPose.getTranslation().y() * 0.0254d, 0d, desiredPose.getRotation().getRadians(), 0d
        }));
    }

    public SimpleMatrix getOptimalInput(int timeStep, SimpleMatrix state, SimpleMatrix desiredState) {
        if(timeStep < K.length) {
            return limitInput(K[timeStep].mult(state.minus(desiredState)));
        }

        return new SimpleMatrix(4, 1);
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

    private static SimpleMatrix getStateCost(int timeStep) {
        return timeStep >= HORIZON_STEP - 1 ? TERMINATION_COST : INTERMEDIARY_STATE_COST;
    }

    public static double getDt() {
        return dt;
    }

    public void setModel(MecanumDriveModel model) {
        this.model = model;
    }
}
