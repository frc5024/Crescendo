package com.team5024.lib.statemachines;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

/**
 * A container for metadata about a state. These are sent to every state's
 * action consumer to provide information about how and when it is being called.
 * 
 * @param <T> {@link StateMachine}'s key datatype
 */
public class StateMetadata<T> {

    private StateMachine<T> parent;

    private T previousState;
    private boolean isFirstRun;

    /**
     * Create StateMetadata
     * 
     * @param parent        State's parent {@link StateMachine}
     * @param previousState The previous state key
     * @param isFirstRun    Is this the first run of the action?
     */
    protected StateMetadata(@Nonnull StateMachine<T> parent, @Nullable T previousState, boolean isFirstRun) {
        this.parent = parent;
        this.previousState = previousState;
        this.isFirstRun = isFirstRun;
    }

    /**
     * Get the {@link StateMachine} object that this state belongs to
     * 
     * @return Paren {@link StateMachine}
     */
    public @Nonnull StateMachine<T> getParent() {
        return parent;
    }

    /**
     * Get the state of the last state to run. Null if this is the first state to be
     * executed.
     * 
     * @return Previous state
     */
    public @Nullable T getPreviousState() {
        return previousState;
    }

    /**
     * Is this the first loop of this action?
     * 
     * @return Is first loop
     */
    public boolean isFirstRun() {
        return isFirstRun;
    }
}