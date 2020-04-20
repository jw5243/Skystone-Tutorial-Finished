package org.firstinspires.ftc.teamcode.main;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.debugging.ComputerDebugger;
import org.firstinspires.ftc.teamcode.debugging.IllegalMessageTypeException;
import org.firstinspires.ftc.teamcode.debugging.MessageOption;
import org.firstinspires.ftc.teamcode.lib.geometry.Line2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;

public class PurePursuitRobot extends Robot {
    private Pose2d finalPose = new Pose2d(10 * 12d, 10 * 12d, new Rotation2d(Math.toRadians(180d), false));
    private Pose2d robotPower;

    private double heading = Robot.getInitialState().get(4);
    private double angularVelocity = 0d;

    @Override
    public void init_debug() {
        super.init_debug();
        robotPower = new Pose2d();
    }

    @Override
    public void loop_debug() {
        super.loop_debug();
        Translation2d displacement = finalPose.getTranslation().translateBy(getFieldPosition().getTranslation().inverse());
        double turnPower = (finalPose.getRotation().getRadians() - heading/*getFieldPosition().getRotation().getRadians()*/) / Math.toRadians(270d);
        robotPower = new Pose2d(displacement.scale(1d / 100d)//.rotateBy(getFieldPosition().getRotation()
                .rotateBy(new Rotation2d(Math.toRadians(-90d), false))/*)*/, new Rotation2d(turnPower, false));
        if(displacement.norm() < 0.01d) {
            robotPower = new Pose2d();
        }

        double maxAngularAcceleration = Math.toRadians(360d * 5d); //rad / s^2
        double maxAngularVelocity = Math.toRadians(360); //rad / s
        double angularAcceleration = maxAngularAcceleration * ((turnPower > 1d ? 1d : turnPower < -1d ? -1d : turnPower) - angularVelocity / maxAngularVelocity);
        angularVelocity += angularAcceleration * getDt();
        heading += angularVelocity * getDt();
        //heading += turnPower * getDt() * 6d;

        try {
            ComputerDebugger.send(MessageOption.LINE.setSendValue(
                    new Line2d(getFieldPosition().getTranslation(), getFieldPosition().getTranslation().translateBy(
                            displacement.scale(20d / displacement.norm())
                    ))
            ));
        } catch (IllegalMessageTypeException e) {
            e.printStackTrace();
        }

        setInput(convertPowerToInput(robotPower));
    }

    @Override
    public Pose2d getFieldPosition() {
        return new Pose2d(getState().get(0) / 0.0254d, getState().get(2) / 0.0254d, new Rotation2d(heading, false));
    }

    public static SimpleMatrix convertPowerToInput(Pose2d power) {
        double x = power.getTranslation().x();
        double y = power.getTranslation().y();
        double turn = power.getRotation().getRadians();

        double frontLeft  = x + y - turn;
        double frontRight = x - y + turn;
        double backLeft   = x - y - turn;
        double backRight  = x + y + turn;

        double highestPower = Math.abs(frontLeft);
        if(highestPower < Math.abs(frontRight)) {
            highestPower = Math.abs(frontRight);
        }

        if(highestPower < Math.abs(backLeft)) {
            highestPower = Math.abs(backLeft);
        }

        if(highestPower < Math.abs(backRight)) {
            highestPower = Math.abs(backRight);
        }

        double normalization = highestPower > 1d ? 1d / highestPower : 1d;

        return new SimpleMatrix(4, 1, false, new double[] {
                (frontLeft + turn) * normalization,
                (frontRight - turn) * normalization,
                (backLeft + turn) * normalization,
                (backRight - turn) * normalization
        });
    }
}
