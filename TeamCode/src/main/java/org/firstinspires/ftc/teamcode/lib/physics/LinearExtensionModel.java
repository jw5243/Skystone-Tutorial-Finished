package org.firstinspires.ftc.teamcode.lib.physics;

public class LinearExtensionModel {
    private final MotorModel                 motorModel;
    private final double                     spoolDiameter; //m

    private double position; //m
    private double velocity; //m / s
    private double acceleration; //m / s^2
    private double jerk; //m / s^3

    private double lastAcceleration; //m / s^2

    public LinearExtensionModel(MotorModel motorModel, double spoolDiameter) {
        this.motorModel      = motorModel;
        this.spoolDiameter   = spoolDiameter;

        setPosition(0d);
        setVelocity(0d);
        setAcceleration(0d);
        setJerk(0d);

        setLastAcceleration(0d);
    }

    public static void main(String... args) {
        final double mechanismWeight   = 44.48d * 12d; //N, 120 lbs
        final double gameElementWeight = 4.48d; //N, 1 lbs
        final double spoolDiameter     = 3d * 0.0254d; //m, 3 in
        LinearExtensionModel linearExtensionModel = new LinearExtensionModel(
                new MotorModel(
                        20d, 12d, 3.36d, 166d,
                        1.3d, 5880d, 0.8d, (motorPosition) -> (mechanismWeight + gameElementWeight) * spoolDiameter * spoolDiameter / 4d,
                        3E-3, 2E-3, 1E-4, 0.05d, 25d),
                spoolDiameter
        );

        System.out.println("t\ty\tv\ta\tj");
        final double dt = 0.001d;
        for(int i = 0; i < 500; i++) {
            linearExtensionModel.update(dt, 12d);
            System.out.print((int)(i * dt * 1000d) / 1000d + "\t");
            System.out.println(linearExtensionModel);
        }
    }

    public void update(double dt, double voltageInput) {
        getMotorModel().update(dt, voltageInput);
        setLastAcceleration(getAcceleration());

        setPosition(getMotorModel().getLinearPosition(getSpoolDiameter()));
        setVelocity(getMotorModel().getLinearVelocity(getSpoolDiameter()));
        setAcceleration(getMotorModel().getLinearAcceleration(getSpoolDiameter()));
        setJerk((getAcceleration() - getLastAcceleration()) / dt);
    }

    @Override
    public String toString() {
        return getPosition() / 0.0254d + "\t" + getVelocity() / 0.0254d + "\t" + getAcceleration() / 0.0254d + "\t" + getJerk() / 0.0254d;
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

    public double getLastAcceleration() {
        return lastAcceleration;
    }

    public void setLastAcceleration(double lastAcceleration) {
        this.lastAcceleration = lastAcceleration;
    }
}
