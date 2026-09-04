package first.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;

import first.robot.subsystems.launcher.LauncherConstants.LauncherStates;
import first.robot.util.LoggedTunableNumber;

public class Launcher implements Mechanism {
    private final LauncherIO io;

    @AutoLogOutput(key="Mechanisms/Launcher/State") private LauncherStates state = LauncherStates.OFF;
    @AutoLogOutput(key="Mechanisms/Launcher/Scoring State") private LauncherStates scoringState = LauncherStates.SELF_DIRECTING;

    @AutoLogOutput(key="Mechanisms/Launcher/Raw Target") private double rawMeanTarget = 0.0;
    @AutoLogOutput(key="Mechanisms/Launcher/Adjust") private double adjust = 0.0;
    @AutoLogOutput(key="Mechanisms/Launcher/True RPS Target") private double meanRPSTarget = 0.0;
    private final LoggedTunableNumber tunableRPS = new LoggedTunableNumber("Launcher/RPS Setpoint", 0.0);

    @AutoLogOutput(key="Mechanisms/Launcher/Error/Minimum Percent") private double minimumErrorPercent = 0.0;

    private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

    public Launcher(LauncherIO io) {
        this.io = io;
    }

    public void logIO() {
        io.updateInputs(inputs);
        Logger.processInputs("Launcher", inputs);

        Logger.recordOutput("Mechanisms/Launcher/Left/RPS Target",
        (meanRPSTarget == 0 ? 0 : ((meanRPSTarget * LauncherConstants.REDUCTION) + (LauncherConstants.RPS_DIFFERENCE / 2)) / LauncherConstants.REDUCTION));
        Logger.recordOutput("Mechanisms/Launcher/Left/RPS", inputs.launcher1Data.velocity());
        Logger.recordOutput("Mechanisms/Launcher/Left/Surface Speed MPS", inputs.launcher1Data.velocity() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);
        
        Logger.recordOutput("Mechanisms/Launcher/Right/RPS Target", 
        (meanRPSTarget == 0 ? 0 : ((meanRPSTarget * LauncherConstants.REDUCTION) - (LauncherConstants.RPS_DIFFERENCE / 2)) / LauncherConstants.REDUCTION));
        Logger.recordOutput("Mechanisms/Launcher/Right/RPS", inputs.launcher2Data.velocity());
        Logger.recordOutput("Mechanisms/Launcher/Right/Surface Speed MPS", inputs.launcher2Data.velocity() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);

        // Logger.recordOutput("Mechanisms/Launcher/Mean/RPS Target", meanRPSTarget);
        Logger.recordOutput("Mechanisms/Launcher/Mean RPS", getMeanRPS());
        Logger.recordOutput("Mechanisms/Launcher/Mean Surface Speed MPS", getMeanRPS() * LauncherConstants.FLYWHEEL_CIRCUMF_METERS);

        Logger.recordOutput("Mechanisms/Launcher/Error/Raw RPS", meanRPSTarget - getMeanRPS());
        Logger.recordOutput("Mechanisms/Launcher/Error/Percent", Math.abs(meanRPSTarget != 0 ? (meanRPSTarget - getMeanRPS()) / meanRPSTarget : 0) * 100);
        // Logger.recordOutput("Mechanisms/Launcher/Error/Minimum Percent", minimumErrorPercent);
    }

    public Command setLauncherRPS(double RPS) { // should NOT be called when in TUNING state
        return run(co -> {
            if (state != LauncherStates.OFF) {
                rawMeanTarget = RPS;
                meanRPSTarget = Math.clamp(rawMeanTarget + adjust, -LauncherConstants.FLYWHEEL_MAX_SPEED_RPS, LauncherConstants.FLYWHEEL_MAX_SPEED_RPS);
                io.setLauncherRPS(meanRPSTarget);
                minimumErrorPercent = Math.abs(meanRPSTarget != 0 ? (meanRPSTarget - getMeanRPS()) / meanRPSTarget : 0) * 100;
                while(minimumErrorPercent > 1) {
                    // functions as a timer, cmd gives up control when it's close to its setpoint
                    if ((meanRPSTarget - getMeanRPS()) / meanRPSTarget < minimumErrorPercent) {minimumErrorPercent = Math.abs((meanRPSTarget - getMeanRPS()) / meanRPSTarget) * 100;}
                    co.yield();
                }
            }
        }).named("LAUNCHER RPS " + (Math.abs(RPS+adjust) < LauncherConstants.FLYWHEEL_MAX_SPEED_RPS ? RPS+adjust : LauncherConstants.FLYWHEEL_MAX_SPEED_RPS));
    }

    public Command applyState(LauncherStates state) {
        return run(co -> {
            this.state = state;
            if(state == LauncherStates.OFF) {
                rawMeanTarget = 0.0;
                meanRPSTarget = 0.0;
                io.setLauncherRPS(meanRPSTarget);
            }
            if(state == LauncherStates.TUNING) {
                co.fork(setScoringState(state));
                while(true) {
                    if (tunableRPS.hasChanged(tunableRPS.hashCode())) rawMeanTarget = tunableRPS.get();
                    meanRPSTarget = Math.clamp(rawMeanTarget + adjust, -LauncherConstants.FLYWHEEL_MAX_SPEED_RPS, LauncherConstants.FLYWHEEL_MAX_SPEED_RPS);
                    io.setLauncherRPS(meanRPSTarget);
                }
            }
        }).named("LAUNCHER " + state);
    }

    public Command setScoringState(LauncherStates state) {
        return Command.noRequirements(co -> {if(state != LauncherStates.OFF) this.scoringState = state;}).named("");
    }

    public Command adjustRPS(double by) {
        return Command.noRequirements(co -> {
            while(true) {
                adjust += by;
                co.fork(setLauncherRPS(rawMeanTarget));
                co.yield();
            }
        }).named("ADJUST LAUNCHER RPS");
    }

    public LauncherStates getState() {
        return state;
    }

    public LauncherStates getScoringState() {
        return scoringState;
    }

    public double getMeanRPS() {
        return ((inputs.launcher1Data.velocity() + inputs.launcher2Data.velocity()) / 2) / LauncherConstants.REDUCTION;
    }

}
