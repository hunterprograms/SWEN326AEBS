package swen326.group4.Audio;

import swen326.group4.Display.AEBSListener;
import swen326.group4.Display.DIDModel;
import javax.sound.sampled.*;
import java.io.File;

public class AuditoryController implements AEBSListener {
    private final File audioFile = new File("resources/warning.wav");
    private volatile boolean isPlaying = false; // volatile ensures thread visibility

    @Override
    public void stateChanged(DIDModel model) {
        if (model.isAlarmActive()) {
            if (!isPlaying) {
                isPlaying = true;
                // Start a new thread to handle the pulsing loop
                Thread audioThread = new Thread(this::runAudioLoop);
                audioThread.setDaemon(true);
                audioThread.start();
            }
        } else {
            isPlaying = false; // Stops the loop in runAudioLoop
        }
    }

    private void runAudioLoop() {
        try {
            while (isPlaying) {
                playSingleChirp();
                // This is the key: The gap between beeps. 
                // Adjust 100 to change the "speed" of the alarm.
                Thread.sleep(100); 
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void playSingleChirp() {
        try (AudioInputStream stream = AudioSystem.getAudioInputStream(audioFile)) {
            AudioFormat baseFormat = stream.getFormat();
            
            // Standardize format to avoid "No line matching" errors
            AudioFormat targetFormat = new AudioFormat(
                AudioFormat.Encoding.PCM_SIGNED,
                baseFormat.getSampleRate(), 16, baseFormat.getChannels(),
                baseFormat.getChannels() * 2, baseFormat.getSampleRate(), false
            );

            try (AudioInputStream convertedStream = AudioSystem.getAudioInputStream(targetFormat, stream)) {
                Clip clip = AudioSystem.getClip();
                clip.open(convertedStream);
                
                clip.start();
                // Wait for exactly the length of your 142ms clip
                Thread.sleep(clip.getMicrosecondLength() / 1000);
                
                clip.close();
            }
        } catch (Exception e) {
            // Fallback to system beep if hardware is busy
            java.awt.Toolkit.getDefaultToolkit().beep();
        }
    }
}