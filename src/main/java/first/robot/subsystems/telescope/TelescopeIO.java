package first.robot.subsystems.telescope;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;

import first.robot.subsystems.telescope.TelescopeConstants.TelescopeInputs;
import first.robot.util.PearadoxTalonFX.MotorData;

public interface TelescopeIO {

    @AutoLog
    public static class TelescopeIOInputs {
        public MotorData pivot1Data;
        public MotorData pivot2Data;
        public MotorData pivot3Data;

        public double pivotAbsEncoderPosition;

        public MotorData arm1Data;
        public MotorData arm2Data;

        public int armServoAppliedPulseWidth;
    }

    public default TelescopeInputs updateInputs() {return new TelescopeInputs();}

    public default void setPivotAngleDeg(double angleDeg) {}

    public default void setPivotAngleDeg(double angleDeg, DoubleSupplier ff) {}

    public default void setArmExtensionIn(boolean isClimbing, double extensionInches) {}

    public default void setArmExtensionIn(boolean isClimbing, double extensionInches, DoubleSupplier ff) {}

    public default void shiftDogs(boolean isClimbing) {}
}
