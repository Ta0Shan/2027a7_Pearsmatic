package first.robot.subsystems.launcher;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import first.robot.Constants;
import first.robot.subsystems.launcher.LauncherConstants.LauncherInputs;
import first.robot.util.PearadoxTalonFX;
import first.robot.util.EnergyTracker.Subsystem;

public abstract class LauncherIOTalonFX implements LauncherIO {
    protected final PearadoxTalonFX launcher1;
    protected final PearadoxTalonFX launcher2;

    protected final VelocityVoltage velocityVoltage;
    protected final CoastOut coastOut;


    public LauncherIOTalonFX() {
        launcher1 = new PearadoxTalonFX(LauncherConstants.LAUNCHER_1_ID,
            Constants.SUPERSTRUCTURE_CAN_BUS,
            LauncherConstants.CONFIG(true),
            Subsystem.LAUNCHER);

        launcher2 = new PearadoxTalonFX(LauncherConstants.LAUNCHER_2_ID,
            Constants.SUPERSTRUCTURE_CAN_BUS,
            LauncherConstants.CONFIG(false),
            Subsystem.LAUNCHER);

        velocityVoltage = new VelocityVoltage(0);
        coastOut = new CoastOut();
    }

    public LauncherInputs updateInputs() {
        return new LauncherInputs(
            launcher1.getData(),
            launcher2.getData()
        );
    }

    public void setLauncherRPS(double rps) {
        if (rps == 0) {
            launcher1.setControl(coastOut);
            launcher2.setControl(coastOut);
            // launcher1.setControl(velocityVoltage.withVelocity(0));
            // launcher2.setControl(velocityVoltage.withVelocity(0));
        } else {
            double motorSetpoint = rps * LauncherConstants.REDUCTION;

            // induce spin for stable shot
            launcher1.setControl(velocityVoltage.withVelocity(motorSetpoint + (LauncherConstants.RPS_DIFFERENCE / 2)));

            launcher2.setControl(velocityVoltage.withVelocity(motorSetpoint - (LauncherConstants.RPS_DIFFERENCE / 2)));
        }
    }
}
