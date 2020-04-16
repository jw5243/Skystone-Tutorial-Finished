package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.lib.physics.LinearExtensionModel;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;

public class LinearlyExtendingRobot extends Robot {
    public static void main(String... args) {
        final double backDriveTorque   = 4.448d * 0.03d * 5d / 1000d; //N m, 0.03 lbs + 5 mm
        final double mechanismWeight   = 4.448d * 16.5d; //N, 16.5 lbs
        final double gameElementWeight = 0d; //N, 0 lbs
        final double spoolDiameter     = 0.55d * 0.0254d; //m, 0.55 in
        LinearExtensionModel linearExtensionModel = new LinearExtensionModel(
                new MotorModel(
                        1d, 12d, 0.519d * 2d, 9.901d,
                        0.4d, 1479.93621621622d, 1d,
                        (motorPosition) -> backDriveTorque,
                        (motorPosition) -> (mechanismWeight + gameElementWeight) * spoolDiameter / 2d,
                        3E-3, 2E-3, 1E-4, 0.05d, 25d),
                spoolDiameter, 0.0025d, 0.002d
        );

        final double dt = 0.001d; //s

        //final double kP = 26d; //V / in
        //final double kI = 1d; //V / (in s)
        //final double kD = 1d; //V s / in

        final double kS = 6d; //V
        final double kV = (12d - kS) / 22d; //V s / in
        final double kA = 0.001d; //V s^2 / in
        final double kP = 1d; //V / in
        final double kI = 0d; //V / (in s)
        final double kD = 0.1d; //V s / in

        double runningSum = 0d;
        double lastError  = 0d;

        double setpoint = 15d; //in

        IMotionProfile motionProfile = new ResidualVibrationReductionMotionProfilerGenerator(0d, setpoint, 22d, 200d);

        System.out.println("t\tV in\ty\tv\ta");
        for(int i = 0; i < 2000; i++) {
            double timeStamp = i * dt;
            //double error = setpoint - linearExtensionModel.getPosition() / 0.0254d;
            double error = motionProfile.getPosition(timeStamp) - linearExtensionModel.getPosition() / 0.0254d;
            runningSum += error * dt;

            double output =
                    kS +
                    kV * motionProfile.getVelocity(timeStamp) +
                    kA * motionProfile.getAcceleration(timeStamp) +
                    kP * error +
                    kI * runningSum +
                    kD * ((error - lastError) / dt - motionProfile.getVelocity(timeStamp));
            output = output < -12d ? 12d : output > 12d ? 12d : output;

            linearExtensionModel.update(dt, output);
            lastError = error;

            System.out.print((int)(timeStamp * 1000d) / 1000d + "\t");
            System.out.print(output + "\t");
            System.out.println(linearExtensionModel);
        }
    }
}
