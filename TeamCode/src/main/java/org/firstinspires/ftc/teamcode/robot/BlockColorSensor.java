package org.firstinspires.ftc.teamcode.robot;

import java.util.HashMap;
import java.util.Objects;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BlockColorSensor {

    public static final HashMap<BlockColor, Double[]> BLOCK_PERCENTS = new HashMap<>();
    public static final HashMap<BlockColor, Double> BLOCK_THRESHOLDS = new HashMap<>();
    public static final Double[] RED_BLOCK_PERCENTS = { 54.7, 28.45, 16.8 };
    public static final Double RED_BLOCK_THRESHOLD = 6.;
    public static final Double[] BLUE_BLOCK_PERCENTS = { 9.58, 23.00, 67.45 };
    public static final Double BLUE_BLOCK_THRESHOLD = 2.;
    public static final Double[] YELLOW_BLOCK_PERCENTS = { 34.30, 52.7, 12.95 };
    public static final Double YELLOW_BLOCK_THRESHOLD = 2.;

    private final Telemetry telemetry;

    private final ColorSensor colorSensor;

    private boolean updatedBlockColor;
    private BlockColor blockColor;

    public BlockColorSensor(HardwareMap hwMap, Telemetry telemetry) {
        colorSensor = hwMap.get(ColorSensor.class, "BlockColorSensor");
        this.telemetry = telemetry;
        updatedBlockColor = false;
        blockColor = BlockColor.NONE;
        BLOCK_PERCENTS.put(BlockColor.RED, RED_BLOCK_PERCENTS);
        BLOCK_PERCENTS.put(BlockColor.BLUE, BLUE_BLOCK_PERCENTS);
        BLOCK_PERCENTS.put(BlockColor.YELLOW, YELLOW_BLOCK_PERCENTS);
        BLOCK_THRESHOLDS.put(BlockColor.RED, RED_BLOCK_THRESHOLD);
        BLOCK_THRESHOLDS.put(BlockColor.BLUE, BLUE_BLOCK_THRESHOLD);
        BLOCK_THRESHOLDS.put(BlockColor.YELLOW, YELLOW_BLOCK_THRESHOLD);
    }

    public int red() {
        return colorSensor.red();
    }
    public int green() {
        return colorSensor.green();
    }
    public int blue() {
        return colorSensor.green();
    }

    public void resetUpdateBlockColor() {
        updatedBlockColor = false;
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

        return colorPercentInRange(rPercent, Objects.requireNonNull(BLOCK_PERCENTS.get(blockColor))[0], BLOCK_THRESHOLDS.get(blockColor)) &&
                colorPercentInRange(gPercent, Objects.requireNonNull(BLOCK_PERCENTS.get(blockColor))[1], BLOCK_THRESHOLDS.get(blockColor)) &&
                colorPercentInRange(bPercent, Objects.requireNonNull(BLOCK_PERCENTS.get(blockColor))[2], BLOCK_THRESHOLDS.get(blockColor));
    }
    private boolean colorPercentInRange(double value, double target, double threshold) {
        return Math.abs(target - value) < threshold;
    }
}
