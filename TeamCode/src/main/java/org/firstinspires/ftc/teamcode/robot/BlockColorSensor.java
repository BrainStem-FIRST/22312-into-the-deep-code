package org.firstinspires.ftc.teamcode.robot;

import java.util.HashMap;
import java.util.Objects;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlockColorSensor {
    public boolean dataMode = false;
    public static final HashMap<BlockColor, Double[]> MAX_BLOCK_PERCENTS = new HashMap<>();
    public static final HashMap<BlockColor, Double[]> MIN_BLOCK_PERCENTS = new HashMap<>();

    public static Double[] RED_BLOCK_PERCENTS_MAX = { 37.5, 22.75, 8.5 };
    public static Double[] RED_BLOCK_PERCENTS_MIN = { 57.5, 42.75, 28.5 };
    // 0.5, 0.3044, 0.1956
    // 0.5274, 0.2927, 0.1799
    public static Double[] BLUE_BLOCK_PERCENTS_MAX = { 23.75, 39., 66.25 };
    public static Double[] BLUE_BLOCK_PERCENTS_MIN = { 3.75, 19., 46.25 };
    // 0.1082, 0.2497, 0.6422
    // 0.1005, 0.2356, 0.664
    public static Double[] YELLOW_BLOCK_PERCENTS_MAX = { 43., 59.25, 27. };
    public static Double[] YELLOW_BLOCK_PERCENTS_MIN = { 23., 39.25, 7. };
    // 0.35, 0.5142, 0.1361
    // 0.3528, 0.5172, 0.1298


    private final ColorSensor colorSensor;

    private boolean updatedBlockColor;
    private BlockColor blockColor;

    public BlockColorSensor(HardwareMap hwMap) {
        colorSensor = hwMap.get(ColorSensor.class, "BlockColorSensor");
        updatedBlockColor = false;
        blockColor = BlockColor.NONE;
        MAX_BLOCK_PERCENTS.put(BlockColor.RED, RED_BLOCK_PERCENTS_MAX);
        MAX_BLOCK_PERCENTS.put(BlockColor.BLUE, BLUE_BLOCK_PERCENTS_MAX);
        MAX_BLOCK_PERCENTS.put(BlockColor.YELLOW, YELLOW_BLOCK_PERCENTS_MAX);
        MIN_BLOCK_PERCENTS.put(BlockColor.RED, RED_BLOCK_PERCENTS_MIN);
        MIN_BLOCK_PERCENTS.put(BlockColor.BLUE, BLUE_BLOCK_PERCENTS_MIN);
        MIN_BLOCK_PERCENTS.put(BlockColor.YELLOW, YELLOW_BLOCK_PERCENTS_MIN);
    }

    public int red() {
        return colorSensor.red();
    }
    public int green() {
        return colorSensor.green();
    }
    public int blue() {
        return colorSensor.blue();
    }

    public void resetUpdateBlockColor() {
        updatedBlockColor = false;
    }

    public void updateTesting(BlockColor blockColor) {
        int red = red(), green = green(), blue = blue();
        int total = red + green + blue;
        double redPercent = red * 1.0 / total, greenPercent = green * 1.0 / total, bluePercent = blue * 1.0 / total;

        Double[] maxPercents = MAX_BLOCK_PERCENTS.get(blockColor), minPercents = MIN_BLOCK_PERCENTS.get(blockColor);
        maxPercents[0] = Math.max(maxPercents[0], redPercent);
        maxPercents[1] = Math.max(maxPercents[1], greenPercent);
        maxPercents[2] = Math.max(maxPercents[2], bluePercent);


    }

    public BlockColor getBlockColor() {
        if (!updatedBlockColor) {
            updatedBlockColor = true;

            // actually find block color
            for (BlockColor blockColor1 : BlockColor.values()) {
                if (blockColor1 == BlockColor.NONE)
                    continue;
                if (hasColor(blockColor1)) {
                    blockColor = blockColor1;
                    return blockColor;
                }
            }
            blockColor = BlockColor.NONE;
        }
        return blockColor;
    }

    public boolean hasColor(BlockColor blockColor) {
        int red = colorSensor.red(), green = colorSensor.green(), blue = colorSensor.blue();
        int sum = red + green + blue;
        double rPercent = red * 100. / sum;
        double gPercent = green * 100. / sum;
        double bPercent = blue * 100. / sum;

        //telemetry.addData("red color percent", rPercent);
        //telemetry.addData("green color percent", gPercent);
        //telemetry.addData("blue color percent", bPercent);

        return colorPercentInRange(rPercent, Objects.requireNonNull(MAX_BLOCK_PERCENTS.get(blockColor))[0], Objects.requireNonNull(MIN_BLOCK_PERCENTS.get(blockColor))[0]) &&
                colorPercentInRange(gPercent, Objects.requireNonNull(MAX_BLOCK_PERCENTS.get(blockColor))[1], Objects.requireNonNull(MIN_BLOCK_PERCENTS.get(blockColor))[1]) &&
                colorPercentInRange(bPercent, Objects.requireNonNull(MAX_BLOCK_PERCENTS.get(blockColor))[2], Objects.requireNonNull(MIN_BLOCK_PERCENTS.get(blockColor))[2]);
    }
    private boolean colorPercentInRange(double num, double min, double max) {
        return num <= max && num >= min;
    }
}
