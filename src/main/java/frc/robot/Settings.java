package frc.robot;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleConsumer;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;

import static frc.robot.Constants.*;

public class Settings implements Sendable {
    public static final DoubleSetting sMaxLinearVelX = new DoubleSetting("Max Linear Velocity X (m/s)", 3);
    public static final DoubleSetting sMaxLinearVelY = new DoubleSetting("Max Linear Velocity Y (m/s)", 3);
    public static final DoubleSetting sMaxAngularVel = new DoubleSetting("Max Angular Velocity (radians/s)", 0.25);
    
    public static final DoubleSetting sF = new DoubleSetting("F Steering Gain", kF);
    public static final DoubleSetting sP = new DoubleSetting("P Steering Gain", kP);
    public static final DoubleSetting sI = new DoubleSetting("I Steering Gain", kI);
    public static final DoubleSetting sD = new DoubleSetting("D Steering Gain", kD);
    public static final DoubleSetting sIntegralZone = new DoubleSetting("Integral Zone Steering Gain", kIntegralZone);
    
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
        private List<DoubleConsumer> collectors = new ArrayList<>();

        private DoubleSetting(String name, double defaultValue) {
            this.name = name;
            currentValue = defaultValue;
        }

        public double get() {
            return currentValue;
        }

        /**
         * Calls the collector with the current value and later values
         */
        public void collect(DoubleConsumer collector) {
            collectors.add(collector);
            collector.accept(currentValue);
        }

        private void handleSet(double value) {
            currentValue = value;
            for (var collector : collectors) {
                collector.accept(value);
            }
        }

        @Override
        public void registerSelf(SendableBuilder builder) {
            builder.addDoubleProperty(this.name, () -> currentValue, this::handleSet);
        }
    }
}
