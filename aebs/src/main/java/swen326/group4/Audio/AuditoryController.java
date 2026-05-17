package swen326.group4.Audio;

import swen326.group4.Display.AEBSListener;
import swen326.group4.Display.DIDModel;
import javax.sound.sampled.*;
import java.io.File;

/**
 * Handles the auditory alerts for the AEBS.
 * Implementation focuses on thread-safety and cross-platform compatibility.
 */
public class AuditoryController implements AEBSListener {
    private final File audioFile = new File("resources/warning.wav");
    private volatile boolean isPlaying = false;
    private Thread audioThread;

    @Override
    public void stateChanged(DIDModel model) {
        // Only start if the alarm is active and not already running
        if (model.isAlarmActive() && !isPlaying) {
            startAlarm();
        } 
        // Stop if the alarm is no longer active
        else if (!model.isAlarmActive() && isPlaying) {
            stopAlarm();
        }
    }

    private synchronized void startAlarm() {
        if (isPlaying) return;
        isPlaying = true;
        audioThread = new Thread(this::runAudioLoop, "AEBS-Audio-Thread");
        audioThread.setDaemon(true); // Ensures the thread dies when the GUI closes
        audioThread.start();
    }

    private synchronized void stopAlarm() {
        isPlaying = false;
        if (audioThread != null) {
            audioThread.interrupt();
            audioThread = null;
        }
    }

    private void runAudioLoop() {
        while (isPlaying && !Thread.currentThread().isInterrupted()) {
            playSingleChirp();
            try {
                // The rhythmic gap between chirps (150ms)
                Thread.sleep(150);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt(); // Restore interrupted status
                break; 
            }
        }
    }

    private void playSingleChirp() {
        if (!audioFile.exists()) {
            System.err.println("Warning: Audio file missing at " + audioFile.getAbsolutePath());
            return;
        }

        try (AudioInputStream ais = AudioSystem.getAudioInputStream(audioFile)) {
            AudioFormat format = ais.getFormat();
            DataLine.Info info = new DataLine.Info(SourceDataLine.class, format);

            // Using the default line for maximum OS compatibility
            try (SourceDataLine line = (SourceDataLine) AudioSystem.getLine(info)) {
                line.open(format);
                line.start();

                byte[] buffer = new byte[1024];
                int bytesRead;
                while ((bytesRead = ais.read(buffer)) != -1) {
                    line.write(buffer, 0, bytesRead);
                }

                line.drain(); // Blocks until the hardware finishes playing
                line.stop();
            }
        } catch (Exception e) {
            // WSL/Linux Fallback: Print the action and trigger a system beep
            System.out.println(">>> [AUDITORY ALERT ACTIVE]"); 
            java.awt.Toolkit.getDefaultToolkit().beep();
        }
    }
}