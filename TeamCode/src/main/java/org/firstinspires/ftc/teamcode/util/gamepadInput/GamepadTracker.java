package org.firstinspires.ftc.teamcode.util.gamepadInput;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.Gamepad;
import java.util.function.*;

public class GamepadTracker {
    private final Gamepad gamepad;

    private int aFrameCount = 0;
    private int bFrameCount = 0;
    private int xFrameCount = 0;
    private int yFrameCount = 0;
    private int dpadUpFrameCount = 0;
    private int dpadDownFrameCount = 0;
    private int dpadLeftFrameCount = 0;
    private int dpadRightFrameCount = 0;
    private int leftBumperFrameCount = 0;
    private int rightBumperFrameCount = 0;
    private int leftTriggerFrameCount = 0;
    private int rightTriggerFrameCount = 0;

    public GamepadTracker(Gamepad gamepad) {
        this.gamepad = gamepad;
    }

    public void update() {
        // Update each button's frame count
        updateButtonFrame(gamepad.a, () -> aFrameCount, (c) -> aFrameCount = c);
        updateButtonFrame(gamepad.b, () -> bFrameCount, (c) -> bFrameCount = c);
        updateButtonFrame(gamepad.x, () -> xFrameCount, (c) -> xFrameCount = c);
        updateButtonFrame(gamepad.y, () -> yFrameCount, (c) -> yFrameCount = c);

        updateButtonFrame(gamepad.dpad_up, () -> dpadUpFrameCount, (c) -> dpadUpFrameCount = c);
        updateButtonFrame(gamepad.dpad_down, () -> dpadDownFrameCount, (c) -> dpadDownFrameCount = c);
        updateButtonFrame(gamepad.dpad_left, () -> dpadLeftFrameCount, (c) -> dpadLeftFrameCount = c);
        updateButtonFrame(gamepad.dpad_right, () -> dpadRightFrameCount, (c) -> dpadRightFrameCount = c);

        updateButtonFrame(gamepad.left_bumper, () -> leftBumperFrameCount, (c) -> leftBumperFrameCount = c);
        updateButtonFrame(gamepad.right_bumper, () -> rightBumperFrameCount, (c) -> rightBumperFrameCount = c);

        // For triggers, consider them "pressed" if they exceed a threshold
        updateButtonFrame(gamepad.left_trigger > 0.1, () -> leftTriggerFrameCount, (c) -> leftTriggerFrameCount = c);
        updateButtonFrame(gamepad.right_trigger > 0.1, () -> rightTriggerFrameCount, (c) -> rightTriggerFrameCount = c);
    }

    private void updateButtonFrame(boolean isPressed, IntSupplier frameCountSupplier, IntConsumer frameCountSetter) {
        if (isPressed) {
            frameCountSetter.accept(frameCountSupplier.getAsInt() + 1);
        } else {
            frameCountSetter.accept(0);
        }
    }

    public Gamepad getGamepad() {
        return gamepad;
    }

    // First-frame checker methods
    public boolean isFirstFrameA() { return aFrameCount == 1; }
    public boolean isFirstFrameB() { return bFrameCount == 1; }
    public boolean isFirstFrameX() { return xFrameCount == 1; }
    public boolean isFirstFrameY() { return yFrameCount == 1; }

    public boolean isFirstFrameDpadUp() { return dpadUpFrameCount == 1; }
    public boolean isFirstFrameDpadDown() { return dpadDownFrameCount == 1; }
    public boolean isFirstFrameDpadLeft() { return dpadLeftFrameCount == 1; }
    public boolean isFirstFrameDpadRight() { return dpadRightFrameCount == 1; }

    public boolean isFirstFrameLeftBumper() { return leftBumperFrameCount == 1; }
    public boolean isFirstFrameRightBumper() { return rightBumperFrameCount == 1; }
    public boolean isFirstFrameLeftTrigger() { return leftTriggerFrameCount == 1; }
    public boolean isFirstFrameRightTrigger() { return rightTriggerFrameCount == 1; }

    // Frame count getters
    public int getAFrameCount() { return aFrameCount; }
    public int getBFrameCount() { return bFrameCount; }
    public int getXFrameCount() { return xFrameCount; }
    public int getYFrameCount() { return yFrameCount; }

    public int getDpadUpFrameCount() { return dpadUpFrameCount; }
    public int getDpadDownFrameCount() { return dpadDownFrameCount; }
    public int getDpadLeftFrameCount() { return dpadLeftFrameCount; }
    public int getDpadRightFrameCount() { return dpadRightFrameCount; }

    public int getLeftBumperFrameCount() { return leftBumperFrameCount; }
    public int getRightBumperFrameCount() { return rightBumperFrameCount; }
    public int getLeftTriggerFrameCount() { return leftTriggerFrameCount; }
    public int getRightTriggerFrameCount() { return rightTriggerFrameCount; }

    // Pressed state checkers
    public boolean isAPressed() { return aFrameCount > 0; }
    public boolean isBPressed() { return bFrameCount > 0; }
    public boolean isXPressed() { return xFrameCount > 0; }
    public boolean isYPressed() { return yFrameCount > 0; }

    public boolean isDpadUpPressed() { return dpadUpFrameCount > 0; }
    public boolean isDpadDownPressed() { return dpadDownFrameCount > 0; }
    public boolean isDpadLeftPressed() { return dpadLeftFrameCount > 0; }
    public boolean isDpadRightPressed() { return dpadRightFrameCount > 0; }

    public boolean isLeftBumperPressed() { return leftBumperFrameCount > 0; }
    public boolean isRightBumperPressed() { return rightBumperFrameCount > 0; }
    public boolean isLeftTriggerPressed() { return leftTriggerFrameCount > 0; }
    public boolean isRightTriggerPressed() { return rightTriggerFrameCount > 0; }
}
