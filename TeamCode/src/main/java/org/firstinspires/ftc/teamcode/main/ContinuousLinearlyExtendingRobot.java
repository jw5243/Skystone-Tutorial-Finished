package org.firstinspires.ftc.teamcode.main;

import org.firstinspires.ftc.teamcode.debugging.ComputerDebugger;
import org.firstinspires.ftc.teamcode.debugging.IllegalMessageTypeException;
import org.firstinspires.ftc.teamcode.debugging.MessageOption;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.motion.IMotionProfile;
import org.firstinspires.ftc.teamcode.lib.motion.ResidualVibrationReductionMotionProfilerGenerator;
import org.firstinspires.ftc.teamcode.lib.physics.LinearExtensionModel;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;

import java.util.function.DoubleToIntFunction;
import java.util.function.IntToDoubleFunction;

public class ContinuousLinearlyExtendingRobot extends Robot {
    private final double slideWeight     = 4.448d * 2d; //N, 2 lbs
    private final double mechanismWeight = 4.448d * 5d; //N, 5 lbs
    private final double spoolDiameter   = 1d * 0.0254d; //m, 1 in
    private final double stageLength     = 18d; //in
    private final int    stageCount      = 3;
    private LinearExtensionModel linearExtensionModel;

    private final double kS = 2.2d; //V
    private final double kV = 0d; //V s / in
    private final double kA = 0d; //V s^2 / in
    private final double kP = 4d; //V / in
    private final double kI = 0.05d; //V / (in s)
    private final double kD = 4d; //V s / in

    private double runningSum = 0d;
    private double lastError  = 0d;

    private double setpoint = 30d; //in

    private IMotionProfile motionProfile;

    @Override
    public void init_debug() {
        super.init_debug();
        DoubleToIntFunction currentStage = (motorPosition) -> (int)(1 + MotorModel.getLinearPosition(motorPosition, spoolDiameter) / (0.0254d * stageLength));
        IntToDoubleFunction effectiveLoad = (stage) -> (stage < 1 ? 0d : mechanismWeight + slideWeight / 2d +
                (stage < stageCount ? slideWeight * (stage - 1) : slideWeight * (stageCount - 1))) * spoolDiameter / 2d;

        linearExtensionModel = new LinearExtensionModel(
                MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, 1, 1d,
                        (motorPosition) -> effectiveLoad.applyAsDouble(currentStage.applyAsInt(motorPosition))),
                spoolDiameter, 0.0025d, 0.002d
        );

        motionProfile = new ResidualVibrationReductionMotionProfilerGenerator(0d, 0d, 22d, 200d);
    }

    @Override
    public void start_debug() {
        super.start_debug();
        try {
            ComputerDebugger.send(MessageOption.CONTINUOUS);
            ComputerDebugger.send(MessageOption.STAGE_COUNT.setSendValue(stageCount));
            ComputerDebugger.send(MessageOption.STAGE_LENGTH.setSendValue(stageLength));
            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue(0d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }

        motionProfile.start();
    }

    @Override
    public void loop_debug() {
        try {
            super.loop_debug();
            double dt = getDt();
            double error = setpoint/*motionProfile.getPosition()*/ - linearExtensionModel.getPosition() / 0.0254d;
            runningSum += error * dt;

            double output = kS +
                            kV * motionProfile.getVelocity() +
                            kA * motionProfile.getAcceleration() +
                            kP * error +
                            kI * runningSum +
                            kD * ((error - lastError) / dt - motionProfile.getVelocity());
            output = output < -12d ? -12d : output > 12d ? 12d : output;

            linearExtensionModel.update(dt, output);
            lastError = error;

            ComputerDebugger.send(MessageOption.LINEAR_POSITION.setSendValue((int)(1000d * linearExtensionModel.getPosition() / 0.0254d) / 1000d));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }
    }
}
