package org.firstinspires.ftc.teamcode.main;

import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.debugging.ComputerDebugger;
import org.firstinspires.ftc.teamcode.debugging.IllegalMessageTypeException;
import org.firstinspires.ftc.teamcode.debugging.MessageOption;
import org.firstinspires.ftc.teamcode.lib.control.MecanumRunnableMPC;
import org.firstinspires.ftc.teamcode.lib.util.Time;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.lib.util.TimeUtil;

import java.util.Arrays;

public class GAMPCTuner {
    private static final Time timeout = new Time(10d, TimeUnits.SECONDS);

    private static final int MAX_GENERATIONS = 10;
    private static final int POPULATION_SIZE = 30;

    private static final double MIN_TUNE_VALUE = 0d;
    private static final double MAX_TUNE_VALUE = 100d;

    private static final int TERMS_TO_TUNE = 6; //6 for state cost + 4 for input cost

    private static final int elitismCount = 2;
    private static final double crossoverProbability = 0.9d;
    private static final double mutationProbability = 0.1d;

    private static double[][] populationValues;

    private int currentGeneration;

    public static void main(String... args) {
        GAMPCTuner tuner = new GAMPCTuner();
        Robot.setUsingComputer(true);
        ComputerDebugger.init(new RobotGAMPC());
        tuner.init();
        tuner.simulateGenerations(50);
    }

    public void init() {
        setPopulationValues(new double[getPopulationSize()][getTermsToTune() + 1]); //Additional value for storing the cost
        setCurrentGeneration(1);
    }

    public void runIteration(int index) {
        MecanumRunnableMPC.setStateCost(SimpleMatrix.diag(Arrays.copyOfRange(getPopulationValues()[index], 0, getPopulationValues()[index].length - 1 /*- 4*/)));
        //MecanumRunnableMPC.setInputCost(SimpleMatrix.diag(Arrays.copyOfRange(getPopulationValues()[index], 6, getPopulationValues()[index].length - 1)).scale(1d / getMaxTuneValue()));

        RobotGAMPC robot = new RobotGAMPC();
        ComputerDebugger.setRobot(robot);
        robot.init_debug();

        ComputerDebugger.send(MessageOption.CLEAR_LOG_POINTS);
        ComputerDebugger.send(MessageOption.CLEAR_MOTION_PROFILE);
        ComputerDebugger.sendMessage();

        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        robot.start_debug();
        boolean failed = false;
        while(!robot.isDone() && TimeUtil.getCurrentRuntime().compareTo(getTimeout()) < 0) {
            try {
                if(robot.getFieldPosition().getTranslation().distance(Robot.getInitialPose().getTranslation()) <  1E-6 && TimeUtil.getCurrentRuntime(TimeUnits.SECONDS) > 2d) {
                    failed = true;
                    break;
                }

                robot.loop_debug();

                ComputerDebugger.send(MessageOption.ROBOT_LOCATION);
                ComputerDebugger.send(MessageOption.LOG_POINT.setSendValue(robot.getFieldPosition().getTranslation()));
                ComputerDebugger.sendMessage();
                Thread.sleep(10);
            } catch (InterruptedException | IllegalMessageTypeException e) {
                e.printStackTrace();
            }
        }

        if(failed) {
            for(int i = 0; i < getPopulationValues()[index].length - 1; i++) {
                getPopulationValues()[index][i] = getRandomTuneValue();
            }
        }

        double distanceAwayFromGoal = robot.getFieldPosition().getTranslation().distance(RobotGAMPC.getPositions().get(RobotGAMPC.getPositions().size() - 1).getTranslation());
        double elapsedTime = (robot.getRuntime() == 0d || failed) ? getTimeout().getTimeValue(TimeUnits.SECONDS) : robot.getRuntime();

        double normalizedDistanceCost = 10d;
        double normalizedTimeCost = 20;

        //Update cost value which is set equal to the time elapsed for the iteration
        getPopulationValues()[index][getPopulationValues()[index].length - 1] =
                normalizedDistanceCost * (distanceAwayFromGoal / 144d) + normalizedTimeCost * (elapsedTime / getTimeout().getTimeValue(TimeUnits.SECONDS));
        System.out.print("Iteration: " + index + "\t");
        System.out.print(Arrays.toString(getPopulationValues()[index]));
        System.out.println("\t Took " + elapsedTime + " seconds to finish");
    }

    public void simulateGeneration() {
        System.out.println("-------------------------------- Generation " + getCurrentGeneration() + " --------------------------------");
        if(getCurrentGeneration() == 1) {
            //Take initial generation to be random values
            for(int i = 0; i < getPopulationValues().length; i++) {
                for(int j = 0; j < getPopulationValues()[i].length - 1; j++) {
                    getPopulationValues()[i][j] = getRandomTuneValue();
                }
            }
        } else {
            int k = getElitismCount();
            Arrays.sort(getPopulationValues(), (o1, o2) -> (int)(o1[getTermsToTune()] - o2[getTermsToTune()]));

            double[][] nextPopulationValues = new double[getPopulationSize()][getTermsToTune() + 1];
            for(int i = 0; i < getElitismCount(); i++) {
                System.arraycopy(getPopulationValues()[i], 0, nextPopulationValues[i], 0, nextPopulationValues[0].length);
            }

            while(k < getPopulationSize()) {
                double generationTransitionType = Math.random();
                if(generationTransitionType <= getCrossoverProbability() && k < getPopulationValues().length - 2) {
                    int index1 = getRandomChromosomeIndex();
                    int index2 = getRandomChromosomeIndex();
                    while(index2 == index1) {
                        System.out.println("Recomputing indices");
                        index2 = getRandomChromosomeIndex();
                    }

                    crossover(nextPopulationValues, k, index1, index2);
                    System.out.println("Crossing over " + index1 + " with " + index2 + "\tReplacing index " + k + " and " + (k + 1));
                    k++;
                } else {//if(generationTransitionType <= getCrossoverProbability() + getMutationProbability()) {
                    int index = getRandomChromosomeIndex();
                    mutation(nextPopulationValues, k, index);
                    System.out.println("Mutating " + index + "\tReplacing index " + k);
                }

                k++;
            }

            setPopulationValues(nextPopulationValues);
            System.out.println("//////////////////////////////////////////////////////////////////////////////////");
        }

        for(int i = 0; i < getPopulationValues().length; i++) {
            runIteration(i);
        }

        setCurrentGeneration(getCurrentGeneration() + 1);
    }

    public void simulateGenerations(int generations) {
        for(int i = 0; i < generations; i++) {
            simulateGeneration();
        }
    }

    public void crossover(double[][] nextPopulationValues, int nextPopulationIndex, int index1, int index2) {
        int crossoverIndex = getRandomGeneIndex();
        double crossoverValue = Math.random();
        nextPopulationValues[nextPopulationIndex] = getPopulationValues()[index1];
        nextPopulationValues[nextPopulationIndex + 1] = getPopulationValues()[index2];
        nextPopulationValues[nextPopulationIndex][crossoverIndex] = crossoverValue * getPopulationValues()[index1][crossoverIndex] + (1d - crossoverValue) * getPopulationValues()[index2][crossoverIndex];
        nextPopulationValues[nextPopulationIndex + 1][crossoverIndex] = (1d - crossoverValue) * getPopulationValues()[index1][crossoverIndex] + crossoverValue * getPopulationValues()[index2][crossoverIndex];
    }

    public void mutation(double[][] nextPopulationValues, int nextPopulationIndex, int index) {
        int mutatedTuneIndex = getRandomGeneIndex(); //Not terms-to-tune plus one since cost should not be mutated
        double mutatedValue = getRandomTuneValue();
        nextPopulationValues[nextPopulationIndex] = getPopulationValues()[index];
        nextPopulationValues[nextPopulationIndex][mutatedTuneIndex] = mutatedValue;
    }

    private double getRandomTuneValue() {
        return (getMaxTuneValue() - getMinTuneValue()) * Math.random() + getMinTuneValue();
    }

    private int getRandomChromosomeIndex() {
        if(getCurrentGeneration() == 1) {
            return (int)(getTermsToTune() * Math.random());
        }

        double totalCost = Arrays.stream(getPopulationValues()).mapToDouble(chromosome -> chromosome[chromosome.length - 1]).sum();
        double totalNonnormalizedProbability = Arrays.stream(getPopulationValues()).mapToDouble(chromosome -> Math.pow(totalCost - chromosome[chromosome.length - 1], 4)).sum();
        double[] selectionProbability = Arrays.stream(getPopulationValues()).mapToDouble(chromosome -> Math.pow(totalCost - chromosome[chromosome.length - 1], 4) / totalNonnormalizedProbability).toArray();
        double randomValue = Math.random();
        double currentProbabilitySum = 0d;
        for(int i = 0; i < selectionProbability.length; i++) {
            if(randomValue < selectionProbability[i] + currentProbabilitySum && randomValue >= currentProbabilitySum) {
                return i;
            }

            currentProbabilitySum += selectionProbability[i];
        }

        return selectionProbability.length - 1;
    }

    private int getRandomGeneIndex() {
        return (int)(getTermsToTune() * Math.random());
    }

    public static int getMaxGenerations() {
        return MAX_GENERATIONS;
    }

    public static int getPopulationSize() {
        return POPULATION_SIZE;
    }

    public static int getTermsToTune() {
        return TERMS_TO_TUNE;
    }

    public static double[][] getPopulationValues() {
        return populationValues;
    }

    public static void setPopulationValues(double[][] populationValues) {
        GAMPCTuner.populationValues = populationValues;
    }

    public static double getMinTuneValue() {
        return MIN_TUNE_VALUE;
    }

    public static double getMaxTuneValue() {
        return MAX_TUNE_VALUE;
    }

    public static double getCrossoverProbability() {
        return crossoverProbability;
    }

    public static double getMutationProbability() {
        return mutationProbability;
    }

    public static int getElitismCount() {
        return elitismCount;
    }

    public static Time getTimeout() {
        return timeout;
    }

    public int getCurrentGeneration() {
        return currentGeneration;
    }

    public void setCurrentGeneration(int currentGeneration) {
        this.currentGeneration = currentGeneration;
    }
}
