// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import java.util.function.Supplier;

import org.wpilib.command3.Command;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;

import first.robot.Constants.CrystalColor;
import first.robot.Constants.SuperstructureStates;
import first.robot.subsystems.endEffector.EE;
import first.robot.subsystems.endEffector.EEConstants.RollerStates;
import first.robot.subsystems.launcher.Launcher;
import first.robot.subsystems.launcher.LauncherConstants.LauncherStates;
import first.robot.subsystems.telescope.Telescope;
import first.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class SuperstructureCommands {

    private final Telescope telescope;
    private final EE endEffector;
    private final Launcher launcher;

    private SuperstructureStates superstructureState = SuperstructureStates.HOME;

    private final InterpolatingDoubleTreeMap rpsLerp = new InterpolatingDoubleTreeMap();
    private final LoggedTunableNumber manualSetpoint = new LoggedTunableNumber("Launcher/Manual Setpoint", 30.0);

    // private final Debouncer scoreDebouncer = new Debouncer(1, DebounceType.kFalling);

    // private final Drive drive;

    public SuperstructureCommands(Telescope telescope, Launcher launcher, EE endEffector) {
        this.telescope = telescope;
        this.launcher = launcher;
        this.endEffector = endEffector;

        rpsLerp.put(14.0, 36.);
        rpsLerp.put(11.0, 32.);
        rpsLerp.put(9.0, 28.);
        rpsLerp.put(5.0, 24.);
        rpsLerp.put(2.0, 20.);
    }

    public Command pause(double seconds) {
        return Command.noRequirements(co -> {co.wait(Seconds.of(seconds));}).named("WAIT");
    }
    
    public Command instantApplyState(SuperstructureStates state) {
        return Command.parallel(
            Command.noRequirements(co -> {superstructureState = state;}).named("SET " + superstructureState.name()),
            telescope.applyState(state.telescopeState),
            endEffector.applyState(state.wristState, state.rollerState),
            launcher.applyState(state.usesLauncher ? (state == SuperstructureStates.TUNING ? LauncherStates.TUNING : launcher.getScoringState()) : LauncherStates.OFF)
        ).named("SET " + state.name());
    }

    public Command applyState(SuperstructureStates state) {
        return Command.parallel(
            instantApplyState(state),
            hold()
        ).named(instantApplyState(state).name());
    }
    
    public Command shuttle(Supplier<Double> distance) {
        return Command.requiring(launcher).executing(co -> {
            co.fork(instantApplyState(SuperstructureStates.LAUNCHER));
            co.awaitAll(
                telescope.applyState(superstructureState.telescopeState),
                launcher.setLauncherRPS(
                    launcher.getState() == LauncherStates.SELF_DIRECTING
                    ? rpsLerp.get(distance.get())
                    : manualSetpoint.get()
                )
            );
        }).named("SHUTTLE");
    }

    public Command score() {
        return Command.requiring(endEffector).executing(co -> {
            co.fork(endEffector.applyState(superstructureState == SuperstructureStates.LAUNCHER ? RollerStates.FAST_REV : RollerStates.REV));
            // co.await(pause(0.5));
            Debouncer scoreDebouncer = new Debouncer(0.3, DebounceType.kFalling);
            while(scoreDebouncer.calculate(endEffector.hasCrystal())) {
                co.yield();
            }
        }).named("SCORE");
    }

    public Command climb() {
        return Command.requiring(telescope).executing(co -> {
            co.await(instantApplyState(SuperstructureStates.CLUMB));
        }).named("CLIMB");
    }

    public Command hold() {
        return Command.noRequirements(co -> {
            while(true) {
                co.yield();
            }
        }).named("HOLD");
    }


    public SuperstructureStates getSuperstructureState() {
        return superstructureState;
    }

    public double getEEAngleFromFloorDeg() {
        return endEffector.getWristAngleDeg() - telescope.getPivotAngleDeg();
    }

    public CrystalColor getCrystalColor() {
        return endEffector.crystalColor();
    }

}
