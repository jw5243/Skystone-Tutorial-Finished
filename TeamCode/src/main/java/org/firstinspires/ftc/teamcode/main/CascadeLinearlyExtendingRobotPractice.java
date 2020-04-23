package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.debugging.ComputerDebugger;
import org.firstinspires.ftc.teamcode.debugging.IllegalMessageTypeException;
import org.firstinspires.ftc.teamcode.debugging.MessageOption;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.lib.physics.LinearExtensionModel;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;

public class CascadeLinearlyExtendingRobotPractice extends Robot {
    private final double mechanismWeight = 4.448d * 16.5d; //N, 16.5 lbs
    private final double spoolDiameter = 2d * 0.0254d; //m, 2 in
    private final double stageLength = 18d; //in
    private final int stageCount = 7;
    private LinearExtensionModel linearExtensionModel;

    private final double kS = 0d; //V
    private final double kV = 0d; //V s / in
    private final double kA = 0d; //V s^2 / in
    private final double kP = 0d;//4d; //V / in
    private final double kI = 0d; //V / (in s)
    private final double kD = 0d;//4d; //V s / in

    private double runningSum = 0d;
    private double lastError = 0d;

    private double setpoint = 15d;//5d; //in

    private IMotionProfile motionProfile;

    @Override
    public void init_debug() {
        super.init_debug();
        try {
            ComputerDebugger.send(MessageOption.CASCADE);
            ComputerDebugger.send(MessageOption.STAGE_COUNT.setSendValue(stageCount));
            ComputerDebugger.send(MessageOption.STAGE_LENGTH.setSendValue(stageLength));
            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue(0d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }

        linearExtensionModel = new LinearExtensionModel(
                MotorModel.generateMotorModel(Motor.GOBILDA_312_RPM, 2, 1d,
                        (motorPosition) -> mechanismWeight * spoolDiameter / 2d),
                spoolDiameter, 0.0025d, 0.002d
        );

        //linearExtensionModel.overridePosition(15d * 0.0254d);

        motionProfile = new ResidualVibrationReductionMotionProfilerGenerator(0d, setpoint, 20d, 10d);
    }

    @Override
    public void start_debug() {
        super.start_debug();
        motionProfile.start();
    }

    @Override
    public void loop_debug() {
        try {
            super.loop_debug();
            double dt = getDt();
            double error = setpoint - linearExtensionModel.getPosition() / 0.0254d;//motionProfile.getPosition() - linearExtensionModel.getPosition() / 0.0254d;
            runningSum += error * dt;

            double output = kS +
                    kV * motionProfile.getVelocity() +
                    kA * motionProfile.getAcceleration() +
                    kP * error +
                    kI * runningSum +
                    kD * ((error - lastError) / dt); //- motionProfile.getVelocity());
            output = output < -12d ? -12d : output > 12d ? 12d : output;

            ComputerDebugger.send(MessageOption.LIFT_INPUT.setSendValue(output));

            linearExtensionModel.update(dt, output);
            lastError = error;

            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue((int)(1000d * linearExtensionModel.getPosition() * 6d / 0.0254d) / 1000d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void sendMotionProfileData() {
        super.sendMotionProfileData();
        try {
            ComputerDebugger.send(MessageOption.LIFT_POSITION.setSendValue((int)(1000d * linearExtensionModel.getPosition() * 6d / 0.0254d) / 1000d));
            ComputerDebugger.send(MessageOption.LIFT_VELOCITY.setSendValue((int)(1000d * linearExtensionModel.getVelocity() * 6d / 0.0254d) / 1000d));
            //ComputerDebugger.send(MessageOption.LIFT_ACCELERATION.setSendValue((int)(1000d * linearExtensionModel.getAcceleration() * 6d / 0.0254d) / 1000d));
            //ComputerDebugger.send(MessageOption.LIFT_JERK.setSendValue((int)(1000d * linearExtensionModel.getJerk() * 6d / 0.0254d) / 1000d));

            ComputerDebugger.send(MessageOption.LIFT_JERK.setSendValue((int)(1000d * motionProfile.getPosition() * 6d) / 1000d));
            //System.out.println(TimeUtil.getCurrentRuntime(TimeUnits.SECONDS) + "\t" + (int)(1000d * linearExtensionModel.getAcceleration() * 6d / 0.0254d) / 1000d);
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }
}