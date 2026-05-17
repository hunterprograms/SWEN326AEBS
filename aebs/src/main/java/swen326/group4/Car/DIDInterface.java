package swen326.group4.Car;

import javax.swing.JFrame;

import swen326.group4.Audio.AuditoryController;
import swen326.group4.Display.DIDController;
import swen326.group4.Display.DIDModel;
import swen326.group4.Display.DIDView;

public class DIDInterface extends JFrame {

    private final DIDModel model;
    private final DIDView view;
    private final DIDController controller;
    private final AuditoryController audio;
    
    public DIDInterface() {
        // 1. Initialize the internal Title
        super("AEBS - Driver Information Display");

        // 2. Instantiate the DIDMVC components
        this.model = new DIDModel();
        this.view = new DIDView(this.model);
        this.controller = new DIDController(this.model);

        // 3. Connect the audio listener
        this.audio = new AuditoryController();
        this.model.addListener(this.audio);
    }

    public void initialize() {
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        this.add(this.view);

    
        this.pack();
        this.setLocationRelativeTo(null);
        this.setVisible(true);
        this.view.startTimer();
    }

    public DIDModel getModel() {
        return this.model;
    }

    public DIDView getView() {
        return this.view;
    }
    
    public DIDController getController() {
        return this.controller;
    }

    public AuditoryController getAudio() {
        return audio;
    }

    public void end() {
        this.view.stopTimer();
        this.audio.stop();
        this.setVisible(false);
        this.dispose();
    }
}
