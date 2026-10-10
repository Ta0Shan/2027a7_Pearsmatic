package first.robot.subsystems.telescope;

import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.simulation.ElevatorSim;
import org.wpilib.simulation.SingleJointedArmSim;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.servohub.ServoHubSim;

import first.robot.Constants;
import first.robot.subsystems.endEffector.EEConstants;
import first.robot.subsystems.telescope.TelescopeConstants.ArmConstants;
import first.robot.subsystems.telescope.TelescopeConstants.PivotConstants;
import first.robot.subsystems.telescope.TelescopeConstants.TelescopeInputs;
import first.robot.util.PhoenixUtil;
import first.robot.util.TiltedElevatorSim;
import first.robot.util.VariableLengthArmSim;

public class TelescopeIOSim extends TelescopeIOTalonFX {
    private final TalonFXSimState pivot1SimState;
    private final TalonFXSimState pivot2SimState;
    private final TalonFXSimState pivot3SimState;

    private final CANcoderSimState absoluteEncoderSimState;

    private final TalonFXSimState arm1SimState;
    private final TalonFXSimState arm2SimState;

    private final ServoHubSim armHubSim;

    private final VariableLengthArmSim pivotPhysicsSim = new VariableLengthArmSim(
        DCMotor.getKrakenX60Foc(3), 
        PivotConstants.REDUCTION,
        estimateMOI(0),
        ArmConstants.CARRIAGE_LENGTH_METERS,
        0,
        Double.POSITIVE_INFINITY,
        ArmConstants.MASS_KG,
        true
    );

    private final TiltedElevatorSim armPhysicsSim = new TiltedElevatorSim(
        DCMotor.getKrakenX60Foc(2),
        ArmConstants.EXTENSION_REDUCTION,
        ArmConstants.CARRIAGE_MASS_KG,
        ArmConstants.CARRIAGE_DRUM_RADIUS_METERS,
        // 0,
        -10,
        ArmConstants.MAX_EXTENSION_METERS,
        true
    );

    public TelescopeIOSim() {
        super();
        pivot1SimState = pivot1.getSimState();
        pivot2SimState = pivot2.getSimState();
        pivot3SimState = pivot3.getSimState();
        absoluteEncoderSimState = absoluteEncoder.getSimState();

        arm1SimState = arm1.getSimState();
        arm2SimState = arm2.getSimState();

        armHubSim = new ServoHubSim(armHub);
        armHubSim.enable();

    }

    @Override
    public TelescopeInputs updateInputs() {
        pivot1SimState.setSupplyVoltage(12);
        pivot2SimState.setSupplyVoltage(12);
        pivot3SimState.setSupplyVoltage(12);
        // pivotPhysicsSim.setInputVoltage(pivot1SimState.getMotorVoltage());
        pivotPhysicsSim.setInputVoltage((pivot1SimState.getMotorVoltage() + pivot2SimState.getMotorVoltage() + pivot3SimState.getMotorVoltage()) / 3);

        arm1SimState.setSupplyVoltage(12);
        arm2SimState.setSupplyVoltage(12);
        // armPhysicsSim.setInputVoltage(arm1SimState.getMotorVoltage());
        armPhysicsSim.setInputVoltage((arm1SimState.getMotorVoltage() + arm2SimState.getMotorVoltage()) / 2);

        pivotPhysicsSim.update(Constants.LOOP_PERIOD_SEC);
        armPhysicsSim.update(Constants.LOOP_PERIOD_SEC);

        pivotPhysicsSim.setCGRadius(estimateCGRadius(armPhysicsSim.getPosition()));
        pivotPhysicsSim.setMOI(estimateMOI(armPhysicsSim.getPosition()));
        armPhysicsSim.setAngleFromHorizontal(pivotPhysicsSim.getAngle());

        pivot1SimState.setRawRotorPosition(Units.radiansToRotations(pivotPhysicsSim.getAngle()) * PivotConstants.REDUCTION);
        pivot2SimState.setRawRotorPosition(Units.radiansToRotations(pivotPhysicsSim.getAngle()) * PivotConstants.REDUCTION);
        pivot3SimState.setRawRotorPosition(Units.radiansToRotations(pivotPhysicsSim.getAngle()) * PivotConstants.REDUCTION);
        pivot1SimState.setRotorVelocity(Units.radiansToRotations(pivotPhysicsSim.getVelocity()) * PivotConstants.REDUCTION);
        pivot2SimState.setRotorVelocity(Units.radiansToRotations(pivotPhysicsSim.getVelocity()) * PivotConstants.REDUCTION);
        pivot3SimState.setRotorVelocity(Units.radiansToRotations(pivotPhysicsSim.getVelocity()) * PivotConstants.REDUCTION);
        absoluteEncoderSimState.setRawPosition(Units.radiansToRotations(pivotPhysicsSim.getAngle()));

        arm1SimState.setRawRotorPosition((armPhysicsSim.getPosition() / ArmConstants.ROTOR_CIRCUMF_METERS) * ArmConstants.EXTENSION_REDUCTION);
        arm2SimState.setRawRotorPosition((armPhysicsSim.getPosition() / ArmConstants.ROTOR_CIRCUMF_METERS) * ArmConstants.EXTENSION_REDUCTION);
        arm1SimState.setRotorVelocity((armPhysicsSim.getVelocity() / ArmConstants.ROTOR_CIRCUMF_METERS) * ArmConstants.EXTENSION_REDUCTION);
        arm2SimState.setRotorVelocity((armPhysicsSim.getVelocity() / ArmConstants.ROTOR_CIRCUMF_METERS) * ArmConstants.EXTENSION_REDUCTION);

        // arm1SimState.setRawRotorPosition((armPhysicsSim.getPosition() / ArmConstants.ROTOR_CIRCUMF_METERS) * (isClimbing ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION));
        // arm2SimState.setRawRotorPosition((armPhysicsSim.getPosition() / ArmConstants.ROTOR_CIRCUMF_METERS) * (isClimbing ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION));
        // arm1SimState.setRotorVelocity((armPhysicsSim.getVelocity() / ArmConstants.ROTOR_CIRCUMF_METERS) * (isClimbing ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION));
        // arm2SimState.setRotorVelocity((armPhysicsSim.getVelocity() / ArmConstants.ROTOR_CIRCUMF_METERS) * (isClimbing ? ArmConstants.CLIMB_REDUCTION : ArmConstants.EXTENSION_REDUCTION));

        return super.updateInputs();

    }

    private double estimateCGRadius(double extensionLength) {
        // CG = sum(md)/sum(m)
        return ((ArmConstants.STATIC_STAGE_MASS_KG * (ArmConstants.STATIC_STAGE_LENGTH_METERS/2))
                + (ArmConstants.CARRIAGE_MASS_KG * (extensionLength + ArmConstants.CARRIAGE_LENGTH_METERS/2))
                + (EEConstants.MASS_KG * (extensionLength + ArmConstants.CARRIAGE_LENGTH_METERS)))
                / (ArmConstants.MASS_KG + EEConstants.MASS_KG);
    }

    private double estimateMOI(double extensionLength) {
        // parallel axis theorem: I_cm + Md^2
        double carriageMOI = ArmConstants.CARRIAGE_BASE_MOI + 
                ArmConstants.CARRIAGE_MASS_KG * Math.pow(extensionLength + (ArmConstants.CARRIAGE_LENGTH_METERS/2), 2);
        
        // effectively a point mass: mr^2
        double wristMOI = EEConstants.MASS_KG * Math.pow(extensionLength + ArmConstants.CARRIAGE_LENGTH_METERS, 2);

        return ArmConstants.STATIC_STAGE_MOI + carriageMOI + wristMOI;
    }
}
