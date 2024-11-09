package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Input {
    private final GamepadTracker gamepadTracker1, gamepadTracker2;

    public Input(Gamepad gamepad1, Gamepad gamepad2) {
        this.gamepadTracker1 = new GamepadTracker(gamepad1);
        this.gamepadTracker2 = new GamepadTracker(gamepad2);
    }

    public GamepadTracker getGamepadTracker1() {
        return gamepadTracker1;
    }
    public GamepadTracker getGamepadTracker2() {
        return gamepadTracker2;
    }

    public void update() {
        gamepadTracker1.update();
        gamepadTracker2.update();
    }
}
