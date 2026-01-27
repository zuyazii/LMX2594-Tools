# GUI User Manual

1. **Launch**: Run `vna-spoofer-gui`. The activity log in the System Status view records the boot event.
2. **Load or set calibration**:
   - Tap **Load Calibration** (upload icon) to reuse an existing profile.
   - Tap **Setup New** (gear icon) to begin a new calibration workflow; follow any later prompts added during integration. The badge on the card reflects readiness.
3. **Configure a sweep**:
   - Enter start/stop frequencies in GHz and choose the number of points.
   - Toggle the S-parameters (S11, S12, S21, S22) to include; the buttons are sized for touch interaction.
4. **Control the run**:
   - Tap **Start Test** (play icon). Inputs lock while the sweep is active and the hint banner confirms progress.
   - Tap **Stop** (square icon) to abort, or **Reset** (circular arrow) to restore default values and parameter selections.
5. **Review data and status**:
   - Use the large segmented buttons to switch between **S Parameter Charts** and **System Status**. Each segment is tablet-ready for thumb control.
   - The charts page shows four cards reserved for magnitude/phase plots. In this Qt build the graphs are rendered by upcoming C++ widgets rather than the older Matplotlib pipeline; the cards remain placeholders until that widget is wired in.
   - The status page displays meta chips for last update, total messages, and the scrollable activity log.
6. **Exit**: Close the window or quit from the operating system. Persisted settings can be added later via Qt's settings APIs.
