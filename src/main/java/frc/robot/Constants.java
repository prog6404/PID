// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {

    public static class Sensors {

        public static class Ports {
            public static final int IR_INTAKE = 0;
            public static final int IR_STORAGE = 1;
            public static final int IR_SHOOTER = 2;
        }
    }

    public static class Motors {

        public static class Ports {
            // PWM
            public static final int DRIVE_RIGHT_FRONT = 0;
            public static final int DRIVE_RIGHT_BACK = 1;
            public static final int DRIVE_LEFT_FRONT = 2;
            public static final int DRIVE_LEFT_BACK = 3;

            // CAN
            public static final int INTAKE = 13;

            public static final int STORAGE = 15;

            public static final int SHOOTER_SHOOTER_RIGHT = 14;
            public static final int SHOOTER_SHOOTER_LEFT = 12;
        }

        public static final class Velocity {
            public static final double INTAKE = 0.7;

            public static final double STORAGE = 0.5;
        }
    }

    public static class OI_Map {

        public static final int PILOT = 0;
        public static final int COPILOT = 1;
        
        public static final int BUTTON_A = 1;
        public static final int BUTTON_B = 2;
        public static final int BUTTON_X = 3;
        public static final int BUTTON_Y = 4;
        public static final int BUTTON_LEFT_BUMPER = 5;
        public static final int BUTTON_RIGHT_BUMPER = 6;

    }

}
