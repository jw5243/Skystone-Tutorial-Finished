package org.firstinspires.ftc.teamcode.lib.physics;

import java.util.function.DoubleSupplier;

public class MotorModel {
    private static final double RPM_TO_RAD_PER_SECOND = Math.PI / 30d;

    private final double gearRatio;

    private final double nominalVoltage; //V
    private final double stallTorque; //N m
    private final double stallCurrent; //A
    private final double freeCurrent; //A
    private final double freeSpeed; //RPM
    private final double efficiency;
    private final double resistance; //ohms

    private final double kV;
    private final double kT;

    private final DoubleSupplier inertia; //kg m^2

    private double currentAngularPosition;
    private double currentAngularVelocity;
    private double lastAngularAcceleration;

    /**
     * Defines the parameters of the motors based on values that can easily be found on motor spec
     * charts, in addition to the structure that is being built. All the parameters in the constructor
     * completely defines the basic motor model.
     *
     * @param gearRatio      Gearing on the motor to increase the torque output and reduce the max speed
     *                       of the output shaft. FTC motors commonly come with an internal gearbox
     *                       that will adjust the free speed already, in which case, set this value
     *                       to the external gear ratio that is compounded to the motor for further
     *                       adjustments of torque and speed. A number greater than one for this parameter
     *                       suggests an increase and torque and reduction in speed.
     * @param nominalVoltage The theoretical max voltage of the motor, which in FTC should always be 12 V.
     * @param stallTorque    The amount of torque supplied if there is no internal nor external gearbox
     *                       for the motor. The output shaft torque is automatically adjusted base on
     *                       the {@code gearRatio} parameter.
     * @param stallCurrent   The amount of current draw from the motor if the motor is stalled.
     * @param freeCurrent    The amount of current draw from the motor if there is nothing attached
     *                       to the motor, thus allowing it to spin freely.
     * @param freeSpeed      The RPM of the motor in the case of nothing being attached to the motor
     *                       and being run at full speed.
     * @param efficiency     The amount of torque output loss due to gears, frictions, and other
     *                       environmental disturbances that cannot exactly be accounted for. Most
     *                       of the FTC motors use experimental values for the stall torque, so the
     *                       efficiency can be assumed to be 100% unless there is an external gearbox.
     *                       A generally good efficiency coefficient is 80%.
     * @param inertia        The amount of "stuff" preventing the motor from running at its free speed.
     *                       As this value can vary depending on the angular position of the motor
     *                       output shaft, this parameter is made a {@code DoubleSupplier}, enabling
     *                       custom inputs for the inertia value.
     *
     * @see DoubleSupplier
     */
    public MotorModel(double gearRatio, double nominalVoltage, double stallTorque,
                      double stallCurrent, double freeCurrent, double freeSpeed, double efficiency,
                      DoubleSupplier inertia) {
        this.gearRatio = gearRatio;
        this.nominalVoltage = nominalVoltage;
        this.stallTorque = stallTorque * gearRatio * efficiency;
        this.stallCurrent = stallCurrent;
        this.freeCurrent = freeCurrent;
        this.freeSpeed = freeSpeed * getRpmToRadPerSecond();
        this.efficiency = efficiency;

        this.resistance = getNominalVoltage() / getStallCurrent();

        this.kV = (getNominalVoltage() - getResistance() * getFreeCurrent()) / getFreeSpeed();
        this.kT = getStallTorque() / getStallCurrent();

        this.inertia = inertia;

        setCurrentAngularPosition(0d);
        setCurrentAngularVelocity(0d);
        setLastAngularAcceleration(0d);
    }

    public static void main(String... args) {
        MotorModel motorModel = new MotorModel(
                20d, 12d, 3.36d, 166d,
                1.3d, 5880d, 0.8d, () -> 2.66E-3d);

        System.out.println("Time\tθ\tω\tα");
        final double dt = 0.00001d;
        for(int i = 1; i < 200; i++) {
            motorModel.update(dt, 12d);
            System.out.print((int)(dt * i * 100000d) / 100000d + "\t");
            System.out.println(motorModel);
        }
    }

    public void update(double dt, double voltageInput) {
        setLastAngularAcceleration(calculateAngularAcceleration(voltageInput));
        setCurrentAngularVelocity(getCurrentAngularVelocity() + getLastAngularAcceleration() * dt);
        setCurrentAngularPosition(getCurrentAngularPosition() + getCurrentAngularVelocity() * dt);
    }

    /**
     * Determine the theoretical output torque of the motor.
     *
     * @param voltageInput
     * @return
     */
    public double calculateTorque(double voltageInput) {
        return getkT() * getEfficiency() * getGearRatio() * (voltageInput - getkV() * getCurrentAngularVelocity() * getGearRatio()) / getResistance();
    }

    public double calculateAngularAcceleration(double voltageInput) {
         return calculateTorque(voltageInput) / getInertia().getAsDouble();
    }

    @Override
    public String toString() {
        //return "Motor Model {θ: " + getCurrentAngularPosition() + ", ω: " +
        //        getCurrentAngularVelocity() + ", α: " + getLastAngularAcceleration() + "}";
        return getCurrentAngularPosition() + "\t" + getCurrentAngularVelocity() + "\t" + getLastAngularAcceleration();
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public double getResistance() {
        return resistance;
    }

    public double getStallTorque() {
        return stallTorque;
    }

    public double getStallCurrent() {
        return stallCurrent;
    }

    public double getFreeCurrent() {
        return freeCurrent;
    }

    public double getFreeSpeed() {
        return freeSpeed;
    }

    public double getEfficiency() {
        return efficiency;
    }

    public double getkV() {
        return kV;
    }

    public double getkT() {
        return kT;
    }

    public DoubleSupplier getInertia() {
        return inertia;
    }

    public double getCurrentAngularVelocity() {
        return currentAngularVelocity;
    }

    public void setCurrentAngularVelocity(double currentAngularVelocity) {
        this.currentAngularVelocity = currentAngularVelocity;
    }

    public double getCurrentAngularPosition() {
        return currentAngularPosition;
    }

    public void setCurrentAngularPosition(double currentAngularPosition) {
        this.currentAngularPosition = currentAngularPosition;
    }

    public static double getRpmToRadPerSecond() {
        return RPM_TO_RAD_PER_SECOND;
    }

    public double getLastAngularAcceleration() {
        return lastAngularAcceleration;
    }

    public void setLastAngularAcceleration(double lastAngularAcceleration) {
        this.lastAngularAcceleration = lastAngularAcceleration;
    }
}
