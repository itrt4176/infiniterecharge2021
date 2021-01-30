package com.irontigers.robot.triggers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;


/**
 * A {@link Button} that gets its state from an {@link XboxController} D-Pad.
 */
public class DPadButton extends Button {
    private final XboxController m_controller;
    private final DPadDirection m_direction;

    public enum DPadDirection {
        UP(0),
        RIGHT(90),
        DOWN(180),
        LEFT(270);

        public final int value;

        DPadDirection(int value) {
            this.value = value;
        }
    }

    public DPadButton(XboxController controller, DPadDirection direction) {
        requireNonNullParam(controller, "controller", "DPadButton");

        m_controller = controller;
        m_direction = direction;
    }

    @Override
    public boolean get() {
        return m_controller.getPOV() == m_direction.value;
    }
}