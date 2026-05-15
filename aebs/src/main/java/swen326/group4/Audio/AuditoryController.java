package swen326.group4.Audio;

import swen326.group4.Display.AEBSListener;
import swen326.group4.Display.DIDModel;
import javax.sound.sampled.*;
import java.io.BufferedInputStream;
import java.io.InputStream;

public class AuditoryController implements AEBSListener {
    private Clip alarmClip;

    public AuditoryController() {
        loadAlarmSound();
    }

    private void loadAlarmSound() {
        try {
            // Load a short .wav beep from your resource folder
            InputStream is = getClass().getResourceAsStream("/resources/frank.wav");
            AudioInputStream ais = AudioSystem.getAudioInputStream(new BufferedInputStream(is));
            alarmClip = AudioSystem.getClip();
            alarmClip.open(ais);
        } catch (Exception e) {
            System.err.println("Auditory Intervention Failure: Could not load sound file.");
        }
    }

    @Override
    public void stateChanged(DIDModel model) {
        if (model.isAlarmActive() && alarmClip != null) {
            if (!alarmClip.isRunning()) {
                alarmClip.loop(Clip.LOOP_CONTINUOUSLY); // Spec 4.1: Continuous alert
            }
        } else {
            if (alarmClip != null && alarmClip.isRunning()) {
                alarmClip.stop(); // Stop immediately when alarmActive is false
            }
        }
    }
}
