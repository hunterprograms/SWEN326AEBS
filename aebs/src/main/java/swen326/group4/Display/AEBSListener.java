package swen326.group4.Display;

/**
 * 
 */
public interface AEBSListener {

    /**
     * Called whenever the DIDModel's state (speed, distance, or braking) is updated.
     * @param model the model instance that changed.
     */
    void stateChanged(DIDModel model);
    
}
