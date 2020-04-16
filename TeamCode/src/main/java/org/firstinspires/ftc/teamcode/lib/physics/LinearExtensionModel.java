package org.firstinspires.ftc.teamcode.lib.physics;

import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;

public class LinearExtensionModel {
    interface WeightFunction<E> {
        double apply(boolean toggleAdditionalWeight, E rotation);
    }

    /**
     * This function returns the weight that the motor must overcome in order to extend. We use a
     * {@code WeightFunction} class as a way to vary the weight of the mechanism depending
     * on if the robot is lifting a game element at the same time as the mechanism. The generic class
     * is used in the case that the linear slides are at an angle, reducing the effective weight that
     * the motor must overcome.
     */
    private final WeightFunction<Rotation2d> mechanismWeight;
    private final MotorModel                 motorModel;
    private final double                     spoolDiameter;

    private double position;
    private double velocity;
    private double acceleration;
    private double jerk;

    public LinearExtensionModel(WeightFunction<Rotation2d> mechanismWeight, MotorModel motorModel, double spoolDiameter) {
        this.mechanismWeight = mechanismWeight;
        this.motorModel      = motorModel;
        this.spoolDiameter   = spoolDiameter;

        setPosition(0d);
        setVelocity(0d);
        setAcceleration(0d);
        setJerk(0d);
    }

    public void update(double dt, double motorPower) {

    }

    public WeightFunction<Rotation2d> getMechanismWeight() {
        return mechanismWeight;
    }

    public MotorModel getMotorModel() {
        return motorModel;
    }

    public double getSpoolDiameter() {
        return spoolDiameter;
    }

    public double getPosition() {
        return position;
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;
    }

    public double getJerk() {
        return jerk;
    }

    public void setJerk(double jerk) {
        this.jerk = jerk;
    }
}
