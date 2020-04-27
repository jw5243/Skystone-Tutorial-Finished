package org.firstinspires.ftc.teamcode.lib.control;

import org.ejml.simple.SimpleEVD;
import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public class Obstacle {
    private static final double LENGTH_SCALE = 144d * 0.0254d; //144 in in meters, field length
    private static final SimpleMatrix C = new SimpleMatrix(6, 6, true, new double[] {
            1, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0
    });

    private static final double ROBOT_RADIUS = Math.sqrt(2d) * 9d * 0.0254d; //in
    private double obstacleRadius; //m

    private Translation2d location;
    private double costFactor;

    /**
     *
     * @param x
     * @param y
     * @param radius inches
     * @param costFactor
     */
    public Obstacle(double x, double y, double radius, double costFactor) {
        this(new Translation2d(x, y), radius, costFactor);
    }

    /**
     *
     * @param location
     * @param radius inches
     * @param costFactor
     */
    public Obstacle(Translation2d location, double radius, double costFactor) {
        setLocation(location);
        setObstacleRadius(radius * 0.0254d);
        setCostFactor(costFactor);
    }

    public double distance(Translation2d other) {
        return getLocation().distance(other) * 0.0254d;
    }

    public double distance(SimpleMatrix state) {
        return distance(new Translation2d(state.get(0) / 0.0254d, state.get(2) / 0.0254d));
    }

    public SimpleMatrix stateRepresentation() {
        return new SimpleMatrix(6, 1, true, new double[] {
                getLocation().x() * 0.0254d, 0d, getLocation().y() * 0.0254d, 0d, 0d, 0d
        });
    }

    /**
     *
     *
     * @param expectedState Expected state (via simulation) as part of planned trajectory.
     * @return
     */
    public SimpleMatrix getLinearCost(SimpleMatrix expectedState) {
        SimpleMatrix stateDisplacement = expectedState.minus(stateRepresentation());
        SimpleMatrix displacement = new SimpleMatrix(2, 1, true, new double[] {
                stateDisplacement.get(0), stateDisplacement.get(2)
        });

        double obstacleDistance = Math.sqrt(displacement.transpose().mult(displacement).get(0));
        displacement = displacement.scale(1d / obstacleDistance);
        double distance = obstacleDistance - getRobotRadius() - getObstacleRadius();

        double a0 = getCostFactor() * Math.exp(distance / getLengthScale());
        double a1 = -a0 / getLengthScale();

        return new SimpleMatrix(6, 1, true, new double[] {
                displacement.get(0), 0, displacement.get(1), 0, 0, 0
        }).scale(a1);
        //return C.mult(stateRepresentation().minus(expectedState)).scale(-2d * getCostFactor() * Math.exp(-distance(expectedState)) / (getLengthScale() * getLengthScale()));
    }

    public SimpleMatrix getQuadraticCost(SimpleMatrix expectedState) {
        SimpleMatrix stateDisplacement = expectedState.minus(stateRepresentation());
        SimpleMatrix displacement = new SimpleMatrix(2, 1, true, new double[] {
                stateDisplacement.get(0), stateDisplacement.get(2)
        });

        double obstacleDistance = Math.sqrt(displacement.transpose().mult(displacement).get(0));
        displacement = displacement.scale(1d / obstacleDistance);
        double distance = obstacleDistance - getRobotRadius() - getObstacleRadius();

        SimpleMatrix ortho = new SimpleMatrix(2, 1, true, new double[] {
                displacement.get(1), -displacement.get(0)
        });

        double a0 = getCostFactor() * Math.exp(distance / getLengthScale());
        double a1 = -a0 / getLengthScale();
        double a2 = -a1 / getLengthScale();

        double b2 = a1 / distance;

        SimpleMatrix Q = displacement.mult(displacement.transpose()).scale(a2).plus(ortho.mult(ortho.transpose()).scale(b2));
        SimpleEVD<SimpleMatrix> eigenDecomposition = Q.eig();
        if(eigenDecomposition.getNumberOfEigenvalues() == 2) {
            SimpleMatrix P = new SimpleMatrix(2, 2, true, new double[] {
                    eigenDecomposition.getEigenVector(0).get(0), eigenDecomposition.getEigenVector(1).get(0),
                    eigenDecomposition.getEigenVector(0).get(1), eigenDecomposition.getEigenVector(1).get(1)
            });

            double eigenvalue1 = eigenDecomposition.getEigenvalue(0).getReal();
            double eigenvalue2 = eigenDecomposition.getEigenvalue(1).getReal();
            eigenvalue1 = eigenvalue1 < 0d ? 0d : eigenvalue1;
            eigenvalue2 = eigenvalue2 < 0d ? 0d : eigenvalue2;

            SimpleMatrix D = new SimpleMatrix(2, 2, true, new double[] {
                    eigenvalue1, 0,
                    0, eigenvalue2
            });

            Q = P.mult(D).mult(P.transpose());
        }

        return new SimpleMatrix(6, 6, true, new double[] {
                Q.get(0, 0), 0, Q.get(0, 1), 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                Q.get(1, 0), 0, Q.get(1, 1), 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
        });
    }

    /*public SimpleMatrix getQuadraticCost(SimpleMatrix expectedState) {
        double epsilon = 1E-9;

        double a = getLengthScale();
        double x = expectedState.get(0);
        double y = expectedState.get(2);

        double dxMinus = a - x;
        double dxPlus  = a + x;

        double P00 = a*a - x*x + y*y + Math.sqrt(Math.pow(a, 16)*Math.exp((4*x*x)/a*a)*(dxMinus*dxMinus + y*y)
                *(dxPlus*dxPlus + y*y))/(Math.pow(a, 8)*Math.exp((2*x*x)/a*a))/(2*x*y + epsilon);
        double P02 = (a*a - x*x + y*y - Math.sqrt(Math.pow(a, 16)*Math.exp((4*x*x)/a*a)*(dxMinus*dxMinus + y*y)
                *(dxPlus*dxPlus + y*y))/(Math.pow(a, 8)*Math.exp((2*x*x)/a*a)))/(2*x*y + epsilon);
        double P20 = 1d;
        double P22 = 1d;

        SimpleMatrix P = new SimpleMatrix(6, 6, true, new double[] {
                P00, 0, P02, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                P20, 0, P22, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
        });

        double invP00 = (Math.pow(a, 8)*Math.exp((2*x*x)/a*a)*x*y)/Math.sqrt(Math.pow(a, 16)*Math.exp((4*x*x)/a*a)*(dxMinus*dxMinus + y*y)* (dxPlus*dxPlus + y*y));
        double invP02 = 1/2d - (Math.pow(a, 8)*Math.exp((2*x*x)/a*a)*(a*a - x*x + y*y))/(2*Math.sqrt(Math.pow(a, 16)*Math.exp((4*x*x)/a*a)*(dxMinus*dxMinus + y*y)*(dxPlus*dxPlus + y*y)));
        double invP20 = -((Math.pow(a, 8)*Math.exp((2*x*x)/a*a)*x*y)/Math.sqrt(Math.pow(a, 16)*Math.exp((4*x*x)/a*a)*(dxMinus*dxMinus + y*y)* (dxPlus*dxPlus + y*y)));
        double invP22 = (1 + (Math.pow(a, 8)*Math.exp((2*x*x)/a*a)*(a*a - x*x + y*y))/Math.sqrt(Math.pow(a, 16)*Math.exp((4*x*x)/a*a)*(dxMinus*dxMinus + y*y)*(dxPlus*dxPlus + y*y)))/2;

        SimpleMatrix inverseP = new SimpleMatrix(6, 6, true, new double[] {
                invP00, 0, invP02, 0, 0, 0,
                0, 0, 0, 0, 0 , 0,
                invP20, 0, invP22, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
        });

        double D00 = (Math.exp((-3*x*x + y*y)/a*a)*(2*Math.pow(a, 8)*Math.exp((2*x*x)/a*a)*(x*x + y*y) -
                2*Math.sqrt(Math.pow(a, 16)*Math.exp((4*x*x)/a*a)*(dxMinus*dxMinus + y*y)*(dxPlus*dxPlus + y*y))))/Math.pow(a, 12);
        double D22 = (2*Math.exp((-3*x*x + y*y)/a*a)*(Math.pow(a, 8)*Math.exp((2*x*x)/a*a)*(x*x + y*y) +
                Math.sqrt(Math.pow(a, 16)*Math.exp((4*x*x)/a*a)*(dxMinus*dxMinus + y*y)*(dxPlus*dxPlus + y*y))))/Math.pow(a, 12);

        D00 = D00 < 0d ? 0d : D00;

        SimpleMatrix D = new SimpleMatrix(6, 6, true, new double[] {
                D00, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, D22, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0
        });

        return P.mult(D).mult(inverseP).scale(getCostFactor());
        //return C.mult(stateRepresentation().minus(expectedState).mult(stateRepresentation().minus(expectedState)
        //        .transpose())).mult(C).scale(2d).minus(C).scale(getCostFactor() * Math.exp(-distance(expectedState)) / Math.pow(getLengthScale(), 4d));
    }*/

    public Translation2d getLocation() {
        return location;
    }

    public void setLocation(Translation2d location) {
        this.location = location;
    }

    public double getCostFactor() {
        return costFactor;
    }

    public void setCostFactor(double costFactor) {
        this.costFactor = costFactor;
    }

    public static double getLengthScale() {
        return LENGTH_SCALE;
    }

    public static double getRobotRadius() {
        return ROBOT_RADIUS;
    }

    public double getObstacleRadius() {
        return obstacleRadius;
    }

    public void setObstacleRadius(double obstacleRadius) {
        this.obstacleRadius = obstacleRadius;
    }
}
