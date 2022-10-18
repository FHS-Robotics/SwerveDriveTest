package frc.robot;

import java.lang.reflect.Field;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

public class Settings implements Sendable {
    public static final DoubleSetting sMaxLinearVelX = new DoubleSetting("Max Linear Velocity X (m/s)", 3);
    public static final DoubleSetting sMaxLinearVelY = new DoubleSetting("Max Linear Velocity Y (m/s)", 3);
    public static final DoubleSetting sMaxAngularVel = new DoubleSetting("Max Angular Velocity (radians/s)", 0.25);
    
    private Settings() {}

    public static void initialize() {
        Shuffleboard.getTab("Settings").add(new Settings());
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        for (var field : Settings.class.getDeclaredFields()) {
            if (isSettingField(field)) {
                SettingType setting;
                try {
                    setting = (SettingType) field.get(null);
                } catch (IllegalAccessException e) {
                    throw new RuntimeException(e);
                }
                setting.registerSelf(builder);
            }
        }
    }

    private static boolean isSettingField(Field field) {
        return SettingType.class.isAssignableFrom(field.getType());
    }

    private static interface SettingType {
        public void registerSelf(SendableBuilder builder);
    }

    public static class DoubleSetting implements SettingType {
        private String name;
        private double currentValue;

        private DoubleSetting(String name, double defaultValue) {
            this.name = name;
            currentValue = defaultValue;
        }

        public double get() {
            return currentValue;
        }

        @Override
        public void registerSelf(SendableBuilder builder) {
            builder.addDoubleProperty(this.name, () -> currentValue, (value) -> currentValue = value);
        }
    }
}
