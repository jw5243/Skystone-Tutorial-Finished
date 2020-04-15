package org.firstinspires.ftc.teamcode.debugging;

import org.firstinspires.ftc.teamcode.lib.geometry.Line2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Translation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.main.AutonomousRobot;
import org.firstinspires.ftc.teamcode.main.Robot;

import java.text.DecimalFormat;

public class ComputerDebugger {
    private static UdpServer udpServer;
    private static StringBuilder messageBuilder;
    private static DecimalFormat formatter;
    private static Robot robot;

    public static void init(final Robot robot) {
        setRobot(robot);
        if(!Robot.isUsingComputer()) {
            return;
        }

        setMessageBuilder(new StringBuilder());
        setFormatter(new DecimalFormat("#.00"));
        setUdpServer(new UdpServer(15026));
        new Thread(getUdpServer()).start();
    }

    public static void main(String[] args) {
        Robot.setUsingComputer(true);
        Robot robot = new AutonomousRobot();
        init(robot);
        robot.init_debug();

        send(MessageOption.CLEAR_LOG_POINTS);
        send(MessageOption.CLEAR_MOTION_PROFILE);

        try {
            Thread.sleep(1000);
        } catch(InterruptedException e) {
            e.printStackTrace();
        }

        robot.start_debug();
        while(true) {
            try {
                robot.loop_debug();

                Thread.sleep(50);

                send(MessageOption.ROBOT_LOCATION);
                send(MessageOption.LOG_POINT.setSendValue(robot.getFieldPosition().getTranslation()));
                sendMessage();
            } catch(InterruptedException | IllegalMessageTypeException e) {
                e.printStackTrace();
            }
        }
    }

    public static void send(final MessageOption messageOption) {
        if(!Robot.isUsingComputer() || messageOption == null) {
            return;
        }

        getMessageBuilder().append(messageOption.getTag());

        if(messageOption.ordinal() == MessageOption.ROBOT_LOCATION.ordinal()) {
            getMessageBuilder().append(getRobot().getFieldPosition());
        } else if(messageOption.ordinal() == MessageOption.KEY_POINT.ordinal()) {
            final Translation2d keyPoint = (Translation2d) (messageOption.getSendValue());
            getMessageBuilder().append(keyPoint);
        } else if(messageOption.ordinal() == MessageOption.LOG_POINT.ordinal()) {
            final Translation2d logPoint = (Translation2d) (messageOption.getSendValue());
            getMessageBuilder().append(logPoint);
        } else if(messageOption.ordinal() == MessageOption.LINE.ordinal()) {
            final Line2d line = (Line2d) (messageOption.getSendValue());
            getMessageBuilder().append(line);
        } else if(messageOption.ordinal() == MessageOption.CLEAR_LOG_POINTS.ordinal()) {
            //
        } else if(messageOption.ordinal() == MessageOption.POSITION.ordinal()) {
            //getMessageBuilder().append(Utilities.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append(getRobot().getFieldPosition());
        } else if(messageOption.ordinal() == MessageOption.VELOCITY.ordinal()) {
            //getMessageBuilder().append(Utilities.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append(Speedometer.getCurrentVelocity());
        } else if(messageOption.ordinal() == MessageOption.ACCELERATION.ordinal()) {
            //getMessageBuilder().append(Utilities.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append(Speedometer.getCurrentAcceleration());
        } else if(messageOption.ordinal() == MessageOption.JERK.ordinal()) {
            //getMessageBuilder().append(Utilities.getCurrentRuntime(TimeUnits.SECONDS)).append(",").append(Speedometer.getCurrentJerk());
        } else if(messageOption.ordinal() == MessageOption.CLEAR_MOTION_PROFILE.ordinal()) {
            //
        } else {
            getUdpServer().close();
        }

        getMessageBuilder().append(MessageOption.getEndMessageTag());
    }

    public static void sendMessage() {
        if(!Robot.isUsingComputer()) {
            return;
        }

        getMessageBuilder().append("CLEAR,%");
        getUdpServer().addMessage(getMessageBuilder().toString());
        //System.out.println(getMessageBuilder().toString());
        setMessageBuilder(new StringBuilder());
    }

    public static UdpServer getUdpServer() {
        return udpServer;
    }

    public static void setUdpServer(UdpServer udpServer) {
        ComputerDebugger.udpServer = udpServer;
    }

    public static StringBuilder getMessageBuilder() {
        return messageBuilder;
    }

    public static void setMessageBuilder(StringBuilder messageBuilder) {
        ComputerDebugger.messageBuilder = messageBuilder;
    }

    public static DecimalFormat getFormatter() {
        return formatter;
    }

    public static void setFormatter(DecimalFormat formatter) {
        ComputerDebugger.formatter = formatter;
    }

    public static Robot getRobot() {
        return robot;
    }

    public static void setRobot(Robot robot) {
        ComputerDebugger.robot = robot;
    }
}
