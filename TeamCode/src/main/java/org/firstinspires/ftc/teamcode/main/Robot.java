package org.firstinspires.ftc.teamcode.main;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.debugging.RobotDebug;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveSLQ;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableLQR;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableSLQ;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumDriveModel;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.lib.util.TimeUtil;

public abstract class Robot implements RobotDebug {
    private static final SimpleMatrix INITIAL_STATE = new SimpleMatrix(6, 1, false, new double[] {
            (12d * 12d - 9d) * 0.0254d, 0d, 48d * 0.0254d, 0d, Math.toRadians(-180d), 0d
            //9d * 0.0254d, 0d, 9d * 0.0254d, 0d, Math.toRadians(0d), 0d
    });

    private static boolean isUsingComputer;

    private TimeProfiler timeProfiler;
    private double dt;

    private static SimpleMatrix state;
    private static SimpleMatrix input;

    private static MecanumDriveModel driveModel;
    private static MecanumDriveMPC mecanumDriveMPC;
    private MecanumRunnableLQR mecanumDriveRunnableLQR;

    private static MecanumDriveSLQ mecanumDriveSLQ;
    private MecanumRunnableSLQ mecanumRunnableSLQ;

    @Override
    public void init_debug() {
        setState(getInitialState());
        setInput(new SimpleMatrix(4, 1, false, new double[] {0d, 0d, 0d, 0d}));
        setTimeProfiler(new TimeProfiler(false));
        setDt(0d);
        setDriveModel(new MecanumDriveModel(
                0.001d, 18.14d, 0.315d, 0.315d * (0.1d * 0.1d + 0.032d * 0.032d) / 2d,
                0.315d * (3d * (0.1d * 0.1d + 0.032d * 0.032d) + 0.05d * 0.05d) / 12d, 0.5613d,
                0.1d / 2d, 7d * 0.0254d, 7d * 0.0254d, 6d * 0.0254d, 6d * 0.0254d,
                MotorModel.generateMotorModel(Motor.GOBILDA_435_RPM, null)));
    }

    @Override
    public void start_debug() {
        getTimeProfiler().start();
        TimeUtil.startTime();
    }

    @Override
    public void loop_debug() {
        setDt(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
        setState(getDriveModel().simulate(getState(), getInput(), getDt()));
    }

    @Override
    public void sendMotionProfileData() {

    }

    @Override
    public Pose2d getFieldPosition() {
        return new Pose2d(getState().get(0) / 0.0254d, getState().get(2) / 0.0254d, new Rotation2d(getState().get(4), false));
    }

    public static boolean isUsingComputer() {
        return isUsingComputer;
    }

    public static void setUsingComputer(boolean isUsingComputer) {
        Robot.isUsingComputer = isUsingComputer;
    }

    public void setDt(double dt) {
        this.dt = dt;
    }

    public double getDt() {
        return dt;
    }

    public TimeProfiler getTimeProfiler() {
        return timeProfiler;
    }

    public void setTimeProfiler(TimeProfiler timeProfiler) {
        this.timeProfiler = timeProfiler;
    }

    public static SimpleMatrix getInitialState() {
        return INITIAL_STATE;
    }

    public static SimpleMatrix getState() {
        return state;
    }

    public static void setState(SimpleMatrix state) {
        Robot.state = state;
    }

    public static MecanumDriveModel getDriveModel() {
        return driveModel;
    }

    public static void setDriveModel(MecanumDriveModel driveModel) {
        Robot.driveModel = driveModel;
    }

    public static SimpleMatrix getInput() {
        return input;
    }

    public static void setInput(SimpleMatrix input) {
        Robot.input = input;
    }

    public static MecanumDriveMPC getMecanumDriveMPC() {
        return mecanumDriveMPC;
    }

    public static void setMecanumDriveMPC(MecanumDriveMPC mecanumDriveMPC) {
        Robot.mecanumDriveMPC = mecanumDriveMPC;
    }

    public MecanumRunnableLQR getMecanumDriveRunnableLQR() {
        return mecanumDriveRunnableLQR;
    }

    public void setMecanumDriveRunnableLQR(MecanumRunnableLQR mecanumDriveRunnableLQR) {
        this.mecanumDriveRunnableLQR = mecanumDriveRunnableLQR;
    }

    public static MecanumDriveSLQ getMecanumDriveSLQ() {
        return mecanumDriveSLQ;
    }

    public static void setMecanumDriveSLQ(MecanumDriveSLQ mecanumDriveSLQ) {
        Robot.mecanumDriveSLQ = mecanumDriveSLQ;
    }

    public MecanumRunnableSLQ getMecanumRunnableSLQ() {
        return mecanumRunnableSLQ;
    }

    public void setMecanumRunnableSLQ(MecanumRunnableSLQ mecanumRunnableSLQ) {
        this.mecanumRunnableSLQ = mecanumRunnableSLQ;
    }
}
