// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot.commands;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.StateMachine;
import org.wpilib.command3.StateMachine.State;
import org.wpilib.command3.Trigger;

import first.robot.Constants.SuperstructureStates;
import first.robot.subsystems.drive.Drive;
import first.robot.subsystems.endEffector.EE;
import first.robot.subsystems.launcher.Launcher;
import first.robot.subsystems.telescope.Telescope;

/** Add your docs here. */
public class StateMachineManager {

    private final SuperstructureCommands superstructure;
    private final DriveCommands drivetrain;

    // Button triggers
    private final Trigger homeTrigger;
    private final Trigger intakeTrigger;
    private final Trigger outtakeTrigger;
    private final Trigger L1Trigger;
    private final Trigger L2Trigger;
    private final Trigger classifierTrigger;
    private final Trigger climbTrigger;

    private final Trigger primaryScoreTrigger;
    private final Trigger secondaryScoreTrigger;

    // Robot State Triggers
    private final Trigger isFront;

    // public StateMachineManager(SuperstructureCommands superstructureCommands,
    //                         DriveCommands driveCommands,
    public StateMachineManager(Telescope telescope,
                                Launcher launcher,
                                EE endEffector,
                                Drive drive,
                                DoubleSupplier throttleX,
                                DoubleSupplier throttleY,
                                DoubleSupplier twist,
                                Trigger home, 
                                Trigger intake,
                                Trigger outtake,
                                Trigger goToL1, 
                                Trigger goToL2, 
                                Trigger goToClassifier, 
                                Trigger goToClimb, 
                                Trigger primaryScore,
                                Trigger secondaryScore
                                ) {

        superstructure = new SuperstructureCommands(telescope, launcher, endEffector);
        drivetrain = new DriveCommands(drive);

        drive.setDefaultCommand(drivetrain.joystickDrive(throttleX, throttleY, twist));

        homeTrigger = home;
        intakeTrigger = intake;
        outtakeTrigger = outtake;
        L1Trigger = goToL1;
        L2Trigger = goToL2;
        classifierTrigger = goToClassifier;
        climbTrigger = goToClimb;
        primaryScoreTrigger = primaryScore;
        secondaryScoreTrigger = secondaryScore;

        isFront = new Trigger(() -> drivetrain.isFront(superstructure::getSuperstructureState));
    }

    public StateMachine teleop() {
        // Defining States
            StateMachine stateMachine = new StateMachine("TELE-OP");

            // mechanism states / starters
            State HOME = stateMachine.addState(superstructure.applyState(SuperstructureStates.HOME)); // resting state
            State INTAKING = stateMachine.addState(superstructure.applyState(SuperstructureStates.INTAKING)); // arm down, intaking
            State OUTTAKING = stateMachine.addState(superstructure.applyState(SuperstructureStates.OUTTAKING)); // arm down, outtaking
            State L1_FRONT = stateMachine.addState(superstructure.applyState(SuperstructureStates.L1_FRONT)); // arm up forwards, prepped for L1
            State L1_BACK = stateMachine.addState(superstructure.applyState(SuperstructureStates.L1_BACK)); // arm up backwards, prepped for L1
            State L2_FRONT = stateMachine.addState(superstructure.applyState(SuperstructureStates.L2_FRONT)); // arm up forwards, prepped for L2
            State L2_BACK = stateMachine.addState(superstructure.applyState(SuperstructureStates.L2_BACK)); // arm up backwards, prepped for L2
            State CLASSIFIER_FRONT = stateMachine.addState(superstructure.applyState(SuperstructureStates.CLASSIFIER_FRONT)); // arm up forwards, prepped for lower classifier
            State CLASSIFIER_BACK = stateMachine.addState(superstructure.applyState(SuperstructureStates.CLASSIFIER_BACK)); // arm up backwards, prepped for upper classifier
            State CLIMB_RAISED = stateMachine.addState(superstructure.applyState(SuperstructureStates.CLIMB_RAISED)); // arm up 90, prepped for climb
            State IDLING = stateMachine.addState(superstructure.hold());

            // alignment states / in-betweens
            State SHUTTLE_ALIGN = stateMachine.addState(Command.parallel(superstructure.shuttle(drivetrain::getDistanceFromClassifier), drivetrain.shuttleAlign()).named("SHUTTLE"));
            State COLORED_ALIGN = stateMachine.addState(drivetrain.align(superstructure::getSuperstructureState, superstructure::getCrystalColor));
            State NEUTRAL_ALIGN = stateMachine.addState(drivetrain.align(superstructure::getSuperstructureState));

            // scoring states / finals
            State SCORE = stateMachine.addState(superstructure.score());
            State CLIMB = stateMachine.addState(superstructure.climb());

        // Binding Triggers
            // we will always go to HOME when homeTrigger is triggered
            stateMachine.setInitialState(HOME);

            // HOME, INTAKE, and OUTTAKE can freely switch to each other
            stateMachine.switchFromAny(HOME, OUTTAKING).to(INTAKING).when(intakeTrigger);
            stateMachine.switchFromAny(HOME, INTAKING, IDLING).to(OUTTAKING).when(outtakeTrigger);
            INTAKING.switchTo(HOME).when(intakeTrigger.negate());
            OUTTAKING.switchTo(HOME).when(outtakeTrigger.negate());
            // you cannot go from any scoring level to INTAKE/OUTTAKE without passing back through HOME



            // based on FRONT or BACK, HOME will go to L1/L2/Classifier on goToL1Trigger/goToL2Trigger triggers
            // L1 and L2 can also swap on if driver misclicks
            stateMachine.switchFromAny(HOME, L2_FRONT, L2_BACK, CLASSIFIER_FRONT, CLASSIFIER_BACK, IDLING)
                    .to(() -> isFront.getAsBoolean() ? L1_FRONT : L1_BACK).when(L1Trigger);
            stateMachine.switchFromAny(HOME, L1_FRONT, L1_BACK, CLASSIFIER_FRONT, CLASSIFIER_BACK, IDLING)
                    .to(() -> isFront.getAsBoolean() ? L2_FRONT : L2_BACK).when(L2Trigger);
            stateMachine.switchFromAny(HOME, L1_FRONT, L1_BACK, L2_FRONT, L2_BACK, IDLING)
                    .to(() -> isFront.getAsBoolean() ? CLASSIFIER_FRONT : CLASSIFIER_BACK).when(classifierTrigger);

            stateMachine.switchFromAny(L1_FRONT, L1_BACK, L2_FRONT, L2_BACK, CLASSIFIER_FRONT, CLASSIFIER_BACK, IDLING)
                    .to(HOME).when(homeTrigger);

            // no matter the direction or scoring state, pressing scoreTrigger will align
            stateMachine.switchFromAny(L1_FRONT, L1_BACK, L2_FRONT, L2_BACK, CLASSIFIER_FRONT, CLASSIFIER_BACK, IDLING)
                        .to(COLORED_ALIGN).when(primaryScoreTrigger.risingEdge());
            // using risingEdge() trigger to make sure it's pressed, not held from a previous state

            stateMachine.switchFromAny(L1_FRONT, L1_BACK, L2_FRONT, L2_BACK, CLASSIFIER_FRONT, CLASSIFIER_BACK, IDLING)
                        .to(NEUTRAL_ALIGN).when(secondaryScoreTrigger.risingEdge());
            // NEUTRAL is mapped to secondaryScore because COLORED earns more points, would change if driver shows preference to one config or the other
            // COLORED will run NEUTRAL if there is no color (which technically shouldn't happen irl)
            
            // if the driver second guesses while aligning, letting go of the score trigger will enter a IDLING state
            // IDLING maintains current setpoints so driver can continue or pivot if they want to
            COLORED_ALIGN.switchTo(IDLING).when(primaryScoreTrigger.negate());
            NEUTRAL_ALIGN.switchTo(IDLING).when(secondaryScoreTrigger.negate());
            // ideally the driver shouldn't second guess but it's always nice to have yk
            
            // once the robot is aligned, switch from alignment state to scoring state
            stateMachine.switchFromAny(COLORED_ALIGN, NEUTRAL_ALIGN).to(SCORE).whenComplete();

            // SHUTTLE is the "scoring" mech for the launcher
            HOME.switchTo(SHUTTLE_ALIGN).when(primaryScoreTrigger.risingEdge());

            // if the driver second guesses while about to shoot, letting go of the score trigger will go back to HOME
            SHUTTLE_ALIGN.switchTo(HOME).when(primaryScoreTrigger.negate());
            
            // once the robot is aligned, switch from alignment state to scoring state
            SHUTTLE_ALIGN.switchTo(SCORE).whenComplete();

            // scoring state will always return to HOME after it's done
            SCORE.switchTo(HOME).whenComplete();



            // when climbTrigger is triggered, we will begin climb sequence with CLIMB_RAISED
            HOME.switchTo(CLIMB_RAISED).when(climbTrigger);
            CLIMB_RAISED.switchTo(CLIMB).when(primaryScoreTrigger);
            // marks CLIMB as the final command
            CLIMB.exitStateMachine().whenComplete();

            return stateMachine;
    }

    public StateMachine functional() {
        StateMachine functional = new StateMachine("FUNCTIONAL");

        State DRIVE_CIRCLE = functional.addState(Command.sequence(
            drivetrain.driveCircle(),
            superstructure.pause(1)
        ).withAutomaticName());
        State SPIN = functional.addState(Command.sequence(
            drivetrain.spin(5),
            superstructure.pause(1)
        ).withAutomaticName());
        State INTAKE_OUTTAKE = functional.addState(Command.sequence(
            superstructure.instantApplyState(SuperstructureStates.INTAKING),
            superstructure.pause(1),
            superstructure.instantApplyState(SuperstructureStates.OUTTAKING),
            superstructure.pause(1)
        ).withAutomaticName());
        State L1_FRONT_BACK = functional.addState(Command.sequence(
            superstructure.instantApplyState(SuperstructureStates.L1_FRONT),
            superstructure.pause(1),
            superstructure.instantApplyState(SuperstructureStates.L1_BACK),
            superstructure.pause(1)
        ).withAutomaticName());
        State L2_FRONT_BACK = functional.addState(Command.sequence(
            superstructure.instantApplyState(SuperstructureStates.L2_FRONT),
            superstructure.pause(1),
            superstructure.instantApplyState(SuperstructureStates.L2_BACK),
            superstructure.pause(1)
        ).withAutomaticName());
        State CLASSIFIER_FRONT_BACK = functional.addState(Command.sequence(
            superstructure.instantApplyState(SuperstructureStates.CLASSIFIER_FRONT),
            superstructure.pause(1),
            superstructure.instantApplyState(SuperstructureStates.CLASSIFIER_BACK),
            superstructure.pause(1)
        ).withAutomaticName());
        State LAUNCHER = functional.addState(Command.sequence(
            superstructure.instantApplyState(SuperstructureStates.LAUNCHER),
            superstructure.shuttle(() -> 5.0),
            superstructure.pause(1)
        ).withAutomaticName());
        State CLIMB_SEQUENCE = functional.addState(Command.sequence(
            superstructure.instantApplyState(SuperstructureStates.CLIMB_RAISED),
            superstructure.pause(1),
            superstructure.instantApplyState(SuperstructureStates.CLUMB)
        ).withAutomaticName());

        functional.setInitialState(DRIVE_CIRCLE);
        // functional.setInitialState(INTAKE_OUTTAKE);

        DRIVE_CIRCLE.switchTo(SPIN).whenComplete();
        SPIN.switchTo(INTAKE_OUTTAKE).whenComplete();
        INTAKE_OUTTAKE.switchTo(L1_FRONT_BACK).whenComplete();
        L1_FRONT_BACK.switchTo(L2_FRONT_BACK).whenComplete();
        L2_FRONT_BACK.switchTo(CLASSIFIER_FRONT_BACK).whenComplete();
        CLASSIFIER_FRONT_BACK.switchTo(LAUNCHER).whenComplete();
        LAUNCHER.switchTo(CLIMB_SEQUENCE).whenComplete();
        CLIMB_SEQUENCE.exitStateMachine().whenComplete();

        return functional;
    }

    public Command tuning() {
        return superstructure.instantApplyState(SuperstructureStates.TUNING);
    }

    public void logAdditionalData() {
        Logger.recordOutput("Mechanisms/Superstructure State", superstructure.getSuperstructureState().name());
        Logger.recordOutput("Mechanisms/End Effector/Wrist/Angle From Floor Deg", superstructure.getEEAngleFromFloorDeg());
    }

}
