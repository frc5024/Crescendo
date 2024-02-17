package io.github.frc5024.libkontrol;

import java.util.function.Consumer;

import javax.annotation.Nonnull;
import javax.annotation.Nullable;

public class StateHandler<T> {
    private StateMachine<T> parent;
    private Consumer<StateMetadata<T>> action;

    /**
     * Create a StateHandler for a given state
     * 
     * @param parent The state machine that owns this object
     * @param action The action to be performed during this state
     */
    public StateHandler(@Nonnull StateMachine<T> parent, @Nonnull Consumer<StateMetadata<T>> action) {
        this.parent = parent;
        this.action = action;
    }

    /**
     * Call the action function
     * 
     * @param isNew        Is this the first run?
     * @param lastStateKey Key of the last state to run
     */
    public void call(boolean isNew, @Nullable T lastStateKey) {
        action.accept(new StateMetadata<T>(parent, lastStateKey, isNew));
    }

}