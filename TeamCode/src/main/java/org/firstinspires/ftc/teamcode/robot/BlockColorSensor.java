package org.firstinspires.ftc.teamcode.robot;

import java.util.HashMap;
import java.util.Objects;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class BlockColorSensor {
    private boolean dataMode = false;

    // stores r, g and b bounds as Double[] for each block color as a new entry in the hashmap
    public static final HashMap<BlockColor, Double[]> MAX_BLOCK_PERCENTS = new HashMap<>();
    public static final HashMap<BlockColor, Double[]> MIN_BLOCK_PERCENTS = new HashMap<>();

    public static Double[] RED_BLOCK_PERCENTS_MIN = { 37.5, 22.75, 8.5 };
    public static Double[] RED_BLOCK_PERCENTS_MAX = { 57.5, 42.75, 28.5 };
    public static Double[] BLUE_BLOCK_PERCENTS_MAX = { 23.75, 39., 66.25 };
    public static Double[] BLUE_BLOCK_PERCENTS_MIN = { 3.75, 19., 46.25 };
    public static Double[] YELLOW_BLOCK_PERCENTS_MAX = { 43., 59.25, 27. };
    public static Double[] YELLOW_BLOCK_PERCENTS_MIN = { 23., 39.25, 7. };


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

    public String update(BlockColor blockColor) {
        if(dataMode)
            updateTesting(blockColor);
        if(blockColor == BlockColor.NONE)
            return "block color none; not printing values";
        Double[] max = MAX_BLOCK_PERCENTS.get(blockColor);
        Double[] min = MIN_BLOCK_PERCENTS.get(blockColor);
        return "max percents: " + max[0] + "," + max[1] + "," + max[2] + " | min percents: " + min[0] + ", " + min[1] + ", " + min[2];
        //return "max percents: " + MAX_BLOCK_PERCENTS.get(blockColor) + " | min percents: " + MIN_BLOCK_PERCENTS.get(blockColor);
    }
    private void updateTesting(BlockColor blockColor) {

        if(blockColor == BlockColor.NONE)
            return;

        int red = red(), green = green(), blue = blue();
        int total = red + green + blue;
        double redPercent = red * 1.0 / total, greenPercent = green * 1.0 / total, bluePercent = blue * 1.0 / total;

        Double[] maxPercents = MAX_BLOCK_PERCENTS.get(blockColor);
        Double[] minPercents = MIN_BLOCK_PERCENTS.get(blockColor);

        maxPercents[0] = Math.max(maxPercents[0], redPercent);
        maxPercents[1] = Math.max(maxPercents[1], greenPercent);
        maxPercents[2] = Math.max(maxPercents[2], bluePercent);
        minPercents[0] = Math.min(maxPercents[0], redPercent);
        minPercents[1] = Math.min(maxPercents[1], greenPercent);
        minPercents[2] = Math.min(maxPercents[2], bluePercent);
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


        return colorPercentInRange(rPercent, MAX_BLOCK_PERCENTS.get(blockColor)[0], MIN_BLOCK_PERCENTS.get(blockColor)[0]) &&
                colorPercentInRange(gPercent, MAX_BLOCK_PERCENTS.get(blockColor)[1], MIN_BLOCK_PERCENTS.get(blockColor)[1]) &&
                colorPercentInRange(bPercent, MAX_BLOCK_PERCENTS.get(blockColor)[2], MIN_BLOCK_PERCENTS.get(blockColor)[2]);
    }
    private boolean colorPercentInRange(double num, double max, double min) {
        return num <= max && num >= min;
    }
    public boolean getDataMode() {
        return dataMode;
    }
    public void setDataMode(boolean dataMode) {
        this.dataMode = dataMode;
    }
}
