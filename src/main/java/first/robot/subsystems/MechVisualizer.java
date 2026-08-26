package first.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;

import first.robot.Constants;
import first.robot.subsystems.endEffector.EEConstants;
import first.robot.subsystems.launcher.LauncherConstants;
import first.robot.subsystems.telescope.TelescopeConstants.ArmConstants;

public class MechVisualizer {

    private final Color8Bit idlerColor = new Color8Bit(Color.GOLDENROD);
    private final Color8Bit staticStageColor = new Color8Bit("#00994C");
    private final Color8Bit carriageLigamentColor = new Color8Bit("#00CC66");
    private final Color8Bit eeColor = new Color8Bit("#33FF99");

    private final LoggedMechanism2d mech = new LoggedMechanism2d(4, 3);

    private final LoggedMechanismRoot2d pivotRoot = mech.getRoot("pivot root", 2, 0.5);

    private final LoggedMechanismLigament2d pivotLigament = pivotRoot.append(
        new LoggedMechanismLigament2d(
            "pivot ligament",
            Units.inchesToMeters(5.5),
            -90,
            20,
            idlerColor
        )
    );

    private final LoggedMechanismLigament2d armStaticLigament1 = pivotLigament.append(
        new LoggedMechanismLigament2d(
            "arm static ligament 1",
            ArmConstants.STATIC_STAGE_LENGTH_METERS-Units.inchesToMeters(6.5),
            90,
            12,
            staticStageColor
        )
    );

    private final LoggedMechanismLigament2d launcherLigament1 = armStaticLigament1.append(
        new LoggedMechanismLigament2d(
            "launcher ligament 1",
            LauncherConstants.FLYWHEEL_DIST_FROM_STATIC_STAGE_METERS,
            90,
            10,
            idlerColor
        )
    );
    private final LoggedMechanismLigament2d flywheelLigament1 = launcherLigament1.append(
        new LoggedMechanismLigament2d(
            "flywheel ligament 1",
            Units.inchesToMeters(3),
            0,
            5,
            new Color8Bit(Color.LIME_GREEN))
    );
    private final LoggedMechanismLigament2d launcherLigament2 = armStaticLigament1.append(
        new LoggedMechanismLigament2d(
            "launcher ligament 2",
            LauncherConstants.FLYWHEEL_DIST_FROM_STATIC_STAGE_METERS - Units.inchesToMeters(3),
            90 + 15,
            10,
            idlerColor
        )
    );
    private final LoggedMechanismLigament2d flywheelLigament2 = launcherLigament2.append(
        new LoggedMechanismLigament2d(
            "flywheel ligament 2",
            Units.inchesToMeters(3),
            0,
            5,
            new Color8Bit(Color.LIME_GREEN)
        )
    );
    private double flywheelPosition;

    private final LoggedMechanismLigament2d armStaticLigament2 = armStaticLigament1.append(
        new LoggedMechanismLigament2d(
            "arm static ligament 2",
            Units.inchesToMeters(6.5),
            0,
            12,
            staticStageColor
        )
    );
    private final double carriageOffset = 5.0;
    private final LoggedMechanismLigament2d carriageLigament = armStaticLigament2.append(
        new LoggedMechanismLigament2d(
            "carriage ligament",
            Units.inchesToMeters(carriageOffset), // at max ArmConstants.MAX_EXTENSION_METERS
            0,
            8,
            carriageLigamentColor
        )
    );
    private final double wristOffset = 7.5;
    private final LoggedMechanismLigament2d wristLigament1 = carriageLigament.append(
        new LoggedMechanismLigament2d(
            "wrist ligament 1",
            EEConstants.LENGTH_METERS,
            EEConstants.STARTING_ANGLE_OFFSET_FROM_PARALLEL_DEG - 90 + wristOffset,
            8,
            eeColor
        )
    );
    private final LoggedMechanismLigament2d rollerLigament1 = wristLigament1.append(
        new LoggedMechanismLigament2d(
            "roller ligament 1",
            Units.inchesToMeters(3),
            0,
            5,
            new Color8Bit(Color.ANTIQUE_WHITE)
        )
    );
    private final LoggedMechanismLigament2d wristLigament2 = carriageLigament.append(
        new LoggedMechanismLigament2d(
            "wrist ligament 2",
            EEConstants.LENGTH_METERS,
            EEConstants.STARTING_ANGLE_OFFSET_FROM_PARALLEL_DEG - 90 - wristOffset,
            8,
            eeColor
        )
    );
    private final LoggedMechanismLigament2d rollerLigament2 = wristLigament2.append(
        new LoggedMechanismLigament2d(
            "roller ligament 2", 
            Units.inchesToMeters(3),
            0,
            5,
            new Color8Bit(Color.ANTIQUE_WHITE)
        )
    );
    private double rollersPosition;

    public MechVisualizer() {
        flywheelPosition = 0.0;
        rollersPosition = 0.0;
    }

    public void updateVis(double pivotAngleDegs,
            double armExtensionInches,
            double wristAngleDegs,
            double launcherRPS,
            double rollersRPS) {
        updateMech(pivotAngleDegs, armExtensionInches, wristAngleDegs, launcherRPS, rollersRPS);
        Logger.recordOutput("Simulation/2d Visualizer", mech);
        Logger.recordOutput("Simulation/3d Components", getTransforms());
    }

    private void updateMech(double pivotAngleDegs,
            double armExtensionInches,
            double wristAngleDegs,
            double launcherRPS,
            double rollersRPS) {
        pivotLigament.setAngle(pivotAngleDegs - 90);
        carriageLigament.setLength(Units.inchesToMeters(armExtensionInches + carriageOffset));
        wristLigament1.setAngle(EEConstants.STARTING_ANGLE_OFFSET_FROM_PARALLEL_DEG - wristAngleDegs + wristOffset);
        wristLigament2.setAngle(EEConstants.STARTING_ANGLE_OFFSET_FROM_PARALLEL_DEG - wristAngleDegs - wristOffset);
        flywheelPosition += Units.rotationsToDegrees(launcherRPS/5) * Constants.LOOP_PERIOD_SEC;
        if (launcherRPS == 0) flywheelPosition = 0;
        flywheelLigament1.setAngle(-flywheelPosition);
        flywheelLigament2.setAngle(flywheelPosition);
        rollersPosition += Units.rotationsToDegrees(rollersRPS/5) * Constants.LOOP_PERIOD_SEC;
        if (rollersRPS == 0) rollersPosition = 0;
        rollerLigament1.setAngle(-rollersPosition);
        rollerLigament2.setAngle(rollersPosition);
    }

    static class VisualizerConstants {
        public static final Translation3d STAGE0_ZERO = new Translation3d(-0.241300, 0, 0.377825);
        public static final Translation3d WRIST_ZERO = new Translation3d(0.535083, 0, 0.238125);
        public static final Translation3d WRIST_OFFSET = WRIST_ZERO.minus(STAGE0_ZERO);
    }

    private Transform3d[] getTransforms() {
        Transform3d stage0 = new Transform3d(VisualizerConstants.STAGE0_ZERO, new Rotation3d(0, -Units.degreesToRadians(pivotLigament.getAngle()+90), 0));
        Transform3d stage1 = stage0.plus(new Transform3d(carriageLigament.getLength()-Units.inchesToMeters(carriageOffset), 0, 0, Rotation3d.kZero));
        Transform3d wrist = stage1.plus(new Transform3d(
                VisualizerConstants.WRIST_OFFSET,
                new Rotation3d(0, -Units.degreesToRadians(wristLigament1.getAngle()-wristOffset+55), 0)));

        return new Transform3d[] {stage0, stage1, wrist};
    }
}
