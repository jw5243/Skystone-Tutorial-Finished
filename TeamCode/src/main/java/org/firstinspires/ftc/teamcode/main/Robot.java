package org.firstinspires.ftc.teamcode.main;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.debugging.ComputerDebugger;
import org.firstinspires.ftc.teamcode.debugging.MessageOption;
import org.firstinspires.ftc.teamcode.debugging.RobotDebug;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveILQR;
import org.firstinspires.ftc.teamcode.lib.control.MecanumDriveMPC;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableLQR;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableMPC;
import org.firstinspires.ftc.teamcode.lib.drivers.Motor;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.physics.MecanumDriveModel;
import org.firstinspires.ftc.teamcode.lib.physics.MotorModel;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.lib.util.TimeUtil;

public abstract class Robot implements RobotDebug {
    private static final SimpleMatrix INITIAL_STATE = new SimpleMatrix(6, 1, false, new double[] {
            //(12d * 12d - 9d) * 0.0254d, 0d, 48d * 0.0254d, 0d, Math.toRadians(-180d), 0d
            9d * 0.0254d, 0d, 9d * 0.0254d, 0d, Math.toRadians(0d), 0d

            //9d * 0.0254d, 0d, 48d * 0.0254d, 0d, Math.toRadians(0d), 0d
            //9d * 0.0254d, 0d, (4d + 8d * 5d) * 0.0254d, 0d, Math.toRadians(0d), 0d
            //9d * 0.0254d, 0d, 40d * 0.0254d, 0d, Math.toRadians(-90d), 0d
    });

    private static final Pose2d INITIAL_POSE = new Pose2d(getInitialState().get(0) / 0.0254d, getInitialState().get(2) / 0.0254d, new Rotation2d(getInitialState().get(4), false));

    private static final Translation2d frontLeftWheel  = new Translation2d(-0.18d, 0.1805d);
    private static final Translation2d frontRightWheel = new Translation2d(0.1805d, 0.18d);
    private static final Translation2d backLeftWheel   = new Translation2d(-0.1805d, -0.15d);
    private static final Translation2d backRightWheel  = new Translation2d(0.18d, -0.1505d);

    private static boolean isUsingComputer = true;
    private boolean stopTimer = false;

    private TimeProfiler timeProfiler;
    private double dt;

    private static SimpleMatrix state;
    private static SimpleMatrix input;
    private static SimpleMatrix wheelPositions;

    private static MecanumDriveModel driveModel;
    private static MecanumDriveILQR mecanumDriveILQR;
    private MecanumRunnableLQR mecanumDriveRunnableLQR;

    private static MecanumDriveMPC mecanumDriveMPC;
    private MecanumRunnableMPC mecanumRunnableMPC;

    @Override
    public void init_debug() {
        setState(getInitialState());
        setInput(new SimpleMatrix(4, 1, false, new double[] {0d, 0d, 0d, 0d}));
        setWheelPositions(new SimpleMatrix(4, 1, false, new double[] {0d, 0d, 0d, 0d}));
        setTimeProfiler(new TimeProfiler(false));
        setDt(0d);
        setDriveModel(new MecanumDriveModel(
                0.001d, 18.4d, 0.315d, 0.315d * (0.1d * 0.1d + 0.032d * 0.032d) / 2d,
                0.315d * (3d * (0.1d * 0.1d + 0.032d * 0.032d) + 0.05d * 0.05d) / 12d, 0.5613d,
                0.1d / 2d, 7d * 0.0254d, 7d * 0.0254d, 6d * 0.0254d, 6d * 0.0254d,
                MotorModel.generateMotorModel(Motor.NEVEREST_20, null)));
    }

    @Override
    public void start_debug() {
        getTimeProfiler().start();
        TimeUtil.startTime();
    }

    int frame = 0;

    @Override
    public void loop_debug() {
        if(!stopTimer) {
            ComputerDebugger.send(MessageOption.TIME);
        }

        setDt(getTimeProfiler().getDeltaTime(TimeUnits.SECONDS, true));
        //setDt(1 / 240d);
        setState(getDriveModel().simulate(getState(), getInput(), getDt()));
        //setState(getDriveModel().simulateDynamics(getState(), getInput(), getDt()));

        /*if(!stopTimer) {
            setWheelPositions(getDriveModel().updateWheelAngularPositions(getWheelPositions(), getState(), getDt()));
            SimpleMatrix positions = getWheelPositions().scale(180d / Math.PI);
            if(frame % 8 == 0) {
                SimpleMatrix relativeState = getState().minus(getInitialState()).plus(new SimpleMatrix(6, 1, false, new double[] {
                        -1.4827d, 0d, -1.809d, 0d, Math.toRadians(0d), 0d
                }));

                double x = getInitialState().get(2) - getState().get(2);*/

                /*System.out.println((int)(frame / 8d) + 1 + "\t" + x + "\t" +
                        relativeState.get(0) + "\t" + (relativeState.get(4) * 180d / Math.PI) + "\t" + positions.get(0) + "\t" +
                        positions.get(1) + "\t" + positions.get(2) + "\t" + positions.get(3));*/

                /*System.out.println((int)(frame / 8d) + 1 + "\t" + relativeState.get(0) + "\t" +
                        relativeState.get(2) + "\t" + (relativeState.get(4) * 180d / Math.PI) + "\t" + positions.get(0) + "\t" +
                        positions.get(1) + "\t" + positions.get(2) + "\t" + positions.get(3));
            }

            frame++;
        }*/
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

    public static MecanumDriveILQR getMecanumDriveILQR() {
        return mecanumDriveILQR;
    }

    public static void setMecanumDriveILQR(MecanumDriveILQR mecanumDriveILQR) {
        Robot.mecanumDriveILQR = mecanumDriveILQR;
    }

    public MecanumRunnableLQR getMecanumDriveRunnableLQR() {
        return mecanumDriveRunnableLQR;
    }

    public void setMecanumDriveRunnableLQR(MecanumRunnableLQR mecanumDriveRunnableLQR) {
        this.mecanumDriveRunnableLQR = mecanumDriveRunnableLQR;
    }

    public static MecanumDriveMPC getMecanumDriveMPC() {
        return mecanumDriveMPC;
    }

    public static void setMecanumDriveMPC(MecanumDriveMPC mecanumDriveMPC) {
        Robot.mecanumDriveMPC = mecanumDriveMPC;
    }

    public MecanumRunnableMPC getMecanumRunnableMPC() {
        return mecanumRunnableMPC;
    }

    public void setMecanumRunnableMPC(MecanumRunnableMPC mecanumRunnableMPC) {
        this.mecanumRunnableMPC = mecanumRunnableMPC;
    }

    public void stopTimer() {
        this.stopTimer = true;
    }

    public static SimpleMatrix getWheelPositions() {
        return wheelPositions;
    }

    public static void setWheelPositions(SimpleMatrix wheelPositions) {
        Robot.wheelPositions = wheelPositions;
    }

    public static Pose2d getInitialPose() {
        return INITIAL_POSE;
    }
}
