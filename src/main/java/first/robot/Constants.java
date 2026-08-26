package first.robot;

import java.util.ArrayList;

import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;

import com.ctre.phoenix6.CANBus;

import first.robot.subsystems.endEffector.EEConstants.RollerStates;
import first.robot.subsystems.endEffector.EEConstants.WristStates;
import first.robot.subsystems.telescope.TelescopeConstants.TelescopeStates;

public class Constants {

    public enum Mode {
        REAL,
        SIM,
        REPLAY
    }

    public static final Mode simMode = Mode.SIM;

    public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
    // public static final Mode currentMode = simMode;

    public static final CANBus SUPERSTRUCTURE_CAN_BUS = CANBus.systemcore(4);
    public static final double LOOP_FREQ_HZ = 50;
    public static final double LOOP_PERIOD_SEC = 1 / LOOP_FREQ_HZ;

    public static enum SuperstructureStates {
        HOME(TelescopeStates.HOME, WristStates.STOWED, RollerStates.IDLE),
        INTAKING(TelescopeStates.DOWN, WristStates.DEPLOYED, RollerStates.FAST_FWD),
        OUTTAKING(TelescopeStates.DOWN, WristStates.DEPLOYED, RollerStates.FAST_REV),
        L1_FRONT(TelescopeStates.L1_FRONT, WristStates.L1_FRONT),
        L1_BACK(TelescopeStates.L1_BACK, WristStates.L1_BACK),
        L2_FRONT(TelescopeStates.L2_FRONT, WristStates.L2_FRONT),
        L2_BACK(TelescopeStates.L2_BACK, WristStates.L2_BACK),
        CLASSIFIER_FRONT(TelescopeStates.CLASSIFIER_FRONT, WristStates.CLASSIFIER_FRONT),
        CLASSIFIER_BACK(TelescopeStates.CLASSIFIER_BACK, WristStates.CLASSIFIER_BACK),
        LAUNCHER(TelescopeStates.LAUNCHER, WristStates.STOWED, RollerStates.IDLE, true),
        CLIMB_RAISED(TelescopeStates.CLIMB_RAISED, WristStates.DEPLOYED),
        CLUMB(TelescopeStates.CLUMB, WristStates.DEPLOYED),
        TUNING(TelescopeStates.TUNING, WristStates.TUNING, RollerStates.TUNING, true);

        public final TelescopeStates telescopeState;
        public final WristStates wristState;
        public final RollerStates rollerState;
        public final boolean usesLauncher;

        private SuperstructureStates(TelescopeStates telescopeState, WristStates wristState, RollerStates rollerState, boolean usesLauncher) {
            this.telescopeState = telescopeState;
            this.wristState = wristState;
            this.rollerState = rollerState;
            this.usesLauncher = usesLauncher;
        }
        private SuperstructureStates(TelescopeStates telescopeState, WristStates wristState, RollerStates rollerState) {
            this(telescopeState, wristState, rollerState, false);
        }
        private SuperstructureStates(TelescopeStates telescopeState, WristStates wristState) {
            this(telescopeState, wristState, RollerStates.IDLE, false);
        }
    }

    public static enum CrystalColor {
        ORANGE(0, 1),
        GREEN(1, 3),
        YELLOW(2, 2),
        PURPLE(4, 0),
        NONE(-1, -1)
        ;

        public final int lowerShaftCCWIndex;
        public final int upperShaftCCWIndex;

        private CrystalColor(int lowerShaftCCWIndex,int upperShaftCCWIndex) {
            this.lowerShaftCCWIndex = lowerShaftCCWIndex;
            this.upperShaftCCWIndex = upperShaftCCWIndex;
        }

        public static CrystalColor getFromHex(String hex) {
            switch(hex) {
                case "#ffa500": return CrystalColor.ORANGE;
                case "#00ff00": return CrystalColor.GREEN;
                case "#ffff00": return CrystalColor.YELLOW;
                case "#800080": return CrystalColor.PURPLE;
                default: return CrystalColor.NONE;
            }
        }
    }


    /** Constants for the field.*/
    public static class FieldConstants {

        private static final Translation2d flipXAcrossCenter(Translation2d translation) {
            return new Translation2d(CENTER.minus(translation).getX(), translation.getY());
        }

        private static final Pose2d flipXAcrossCenter(Pose2d pose) {
            return new Pose2d(CENTER.minus(pose.getTranslation()).getX(), pose.getY(), pose.getRotation().plus(Rotation2d.kCCW_90deg.minus(pose.getRotation()).times(2)));
        }

        // OVERALL CONSTANTS
        public static final double FIELD_LENGTH = Units.inchesToMeters(648);
        public static final double FIELD_WIDTH = Units.inchesToMeters(324);

        private static final double ROBOT_LENGTH_WITH_BUMPERS = Units.inchesToMeters(35.5);
        private static final double SHORT_OFFSET = Units.inchesToMeters(2);
        private static final double LONG_OFFSET = Units.inchesToMeters(6);
        public static final Transform2d ALIGN_OFFSET_SHORT =
            new Transform2d(new Translation2d((ROBOT_LENGTH_WITH_BUMPERS)/2+SHORT_OFFSET, Rotation2d.k180deg), Rotation2d.kZero);
        public static final Transform2d ALIGN_OFFSET_LONG =
            new Transform2d(new Translation2d((ROBOT_LENGTH_WITH_BUMPERS/2)+LONG_OFFSET, Rotation2d.k180deg), Rotation2d.kZero);
        
        public static final double MINE_OUTER_WIDTH = Units.inchesToMeters(50.75);
        public static final double MINE_PILLAR_LENGTH = Units.inchesToMeters(12.0);

        public static final Pose2d ORIGIN = new Pose2d();
        public static final Translation2d CENTER = ORIGIN.getTranslation();
        // CENTER-RED field orientation (red on left side):
            // +x = right, -x = left
            // +y = up, -y = down
        // WALL-BLUE field orientation (red on left side):
            // +x = left, -x = right
            // +y = down, -y = up

        /** Constants for the red side of the field. */
        public static class BlueFieldConstants {
            // CAVE
            public static final Translation2d CAVE_CENTER = CENTER.plus(new Translation2d(Units.inchesToMeters(162.591737), 0.));
            public static final Pose2d[] LOWER_SHAFT_FACES = new Pose2d[8];
            static {
                for (int i = 0; i < LOWER_SHAFT_FACES.length; i++) {
                    Rotation2d angle = new Rotation2d(Units.degreesToRadians(45 * i + 22.5));
                    LOWER_SHAFT_FACES[i] = new Pose2d(CAVE_CENTER.plus(new Translation2d(Units.inchesToMeters(32.50 + 10), angle)), angle.minus(Rotation2d.k180deg));
                }
            }
            public static final Pose2d[] UPPER_SHAFT_VERTICES = new Pose2d[4];
            static {
                for (int i = 0; i < UPPER_SHAFT_VERTICES.length; i++) {
                    Rotation2d angle = new Rotation2d(Units.degreesToRadians(90 * i + 45));
                    UPPER_SHAFT_VERTICES[i] = new Pose2d(CAVE_CENTER.plus(new Translation2d(Units.inchesToMeters(12.176912 + 33.917166), angle)), angle.minus(Rotation2d.k180deg));
                }
            }

            // CLASSIFIER
            public static final Translation2d CLASSIFIER_SOURCE_CORNER = CENTER.plus(new Translation2d(Units.inchesToMeters(324.797850), -Units.inchesToMeters(25.777159)));
            public static final Translation2d CLASSIFIER_MINE_CORNER = CLASSIFIER_SOURCE_CORNER.minus(new Translation2d(0, Units.inchesToMeters(62)));
            public static final Pose2d CLASSIFIER_CENTER = new Pose2d(CLASSIFIER_SOURCE_CORNER.plus(CLASSIFIER_MINE_CORNER.minus(CLASSIFIER_SOURCE_CORNER).div(2)), Rotation2d.kZero);
            public static final Translation2d CLASSIFIER_AIM_TARGET = CLASSIFIER_CENTER.getTranslation().minus(new Translation2d(2, 0));

            // STATION
            public static final Translation2d SOURCE_WALL_CORNER = CENTER.plus(new Translation2d(-Units.inchesToMeters(255.446748), Units.inchesToMeters(159.593244)));
            public static final Translation2d SOURCE_DS_CORNER = CENTER.plus(new Translation2d(-Units.inchesToMeters(324.526445), Units.inchesToMeters(119.742799)));
            public static final Pose2d SOURCE_CENTER = new Pose2d(SOURCE_WALL_CORNER.plus(SOURCE_DS_CORNER.minus(SOURCE_WALL_CORNER).div(2)), SOURCE_DS_CORNER.minus(SOURCE_WALL_CORNER).getAngle().plus(Rotation2d.kCCW_90deg));

            // MINE
            public static final Translation2d MINE_CENTER_CORNER = CENTER.plus(new Translation2d(Units.inchesToMeters(125.785719), -Units.inchesToMeters(107.027159)));
            public static final Translation2d MINE_DS_CORNER = CENTER.plus(new Translation2d(Units.inchesToMeters(268.535719), -Units.inchesToMeters(107.027159)));
            public static final Pose2d MINE_CENTER = new Pose2d(MINE_CENTER_CORNER.plus(new Translation2d(MINE_DS_CORNER.minus(MINE_CENTER_CORNER).div(2).getX(), -(MINE_PILLAR_LENGTH + (MINE_OUTER_WIDTH / 2.)))), Rotation2d.kZero);
        }

        /** Constants for the red side of the field.
         * <p> NOTE: this just takes the blue measurements and flips them across the Y axis.
         */
        public static class RedFieldConstants {
            // CAVE
            public static final Translation2d CAVE_CENTER = flipXAcrossCenter(BlueFieldConstants.CAVE_CENTER);
            public static final Pose2d[] LOWER_SHAFT_FACES = new Pose2d[BlueFieldConstants.LOWER_SHAFT_FACES.length];
            static {
                for (int i = 0; i < LOWER_SHAFT_FACES.length; i++) {
                    LOWER_SHAFT_FACES[i] = flipXAcrossCenter(BlueFieldConstants.LOWER_SHAFT_FACES[LOWER_SHAFT_FACES.length - 1 - i]);
                }
            }
            public static final Pose2d[] UPPER_SHAFT_VERTICES = new Pose2d[BlueFieldConstants.UPPER_SHAFT_VERTICES.length];
            static {
                for (int i = 0; i < UPPER_SHAFT_VERTICES.length; i++) {
                    UPPER_SHAFT_VERTICES[i] = flipXAcrossCenter(BlueFieldConstants.UPPER_SHAFT_VERTICES[UPPER_SHAFT_VERTICES.length - 1 - i]);
                }
            }

            // CLASSIFIER
            public static final Translation2d CLASSIFIER_SOURCE_CORNER = flipXAcrossCenter(BlueFieldConstants.CLASSIFIER_SOURCE_CORNER);
            public static final Translation2d CLASSIFIER_MINE_CORNER = flipXAcrossCenter(BlueFieldConstants.CLASSIFIER_MINE_CORNER);
            public static final Pose2d CLASSIFIER_CENTER = flipXAcrossCenter(BlueFieldConstants.CLASSIFIER_CENTER);
            public static final Translation2d CLASSIFIER_AIM_TARGET = flipXAcrossCenter(BlueFieldConstants.CLASSIFIER_AIM_TARGET);

            // STATION
            public static final Translation2d SOURCE_WALL_CORNER = flipXAcrossCenter(BlueFieldConstants.SOURCE_WALL_CORNER);
            public static final Translation2d SOURCE_DS_CORNER = flipXAcrossCenter(BlueFieldConstants.SOURCE_DS_CORNER);
            public static final Pose2d SOURCE_CENTER = flipXAcrossCenter(BlueFieldConstants.SOURCE_CENTER);

            // MINE
            public static final Translation2d MINE_CENTER_CORNER = flipXAcrossCenter(BlueFieldConstants.MINE_CENTER_CORNER);
            public static final Translation2d MINE_DS_CORNER = flipXAcrossCenter(BlueFieldConstants.MINE_DS_CORNER);
            public static final Pose2d MINE_CENTER = flipXAcrossCenter(BlueFieldConstants.MINE_CENTER);
        }

        public static final Pose2d[] getValidGoal(boolean isL1, Pose2d[] shaftList, CrystalColor color) {
            ArrayList<Pose2d> validShafts = new ArrayList<Pose2d>();
            int colorIndex = isL1 ? color.lowerShaftCCWIndex : color.upperShaftCCWIndex;
            for (int i = 0; i < shaftList.length; i++) {
                if (i % 4 == (colorIndex)) {
                    validShafts.add(shaftList[i]);
                }
            }
            return validShafts.toArray(new Pose2d[] {});
        }
    }
}
