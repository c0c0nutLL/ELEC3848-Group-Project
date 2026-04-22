# Door Servo Feedback Probe

This directory contains a small bench-test sketch for checking whether the planned cargo-door servo exposes any readable feedback signal.

Important practical note:

- a standard 3-wire hobby servo usually has internal position feedback for its own control loop
- that internal feedback is usually a potentiometer inside the servo
- in most normal hobby servos, Arduino cannot read that feedback directly
- so in normal project use, you should assume the door servo does **not** provide a readable external encoder

You can only read real servo-position feedback if at least one of these is true:

- the servo model provides a separate analog feedback wire
- the servo is a smart / bus servo with a telemetry protocol
- the datasheet explicitly says position feedback is externally available

Without one of those, the best you can do is:

- command the servo to move
- visually confirm motion
- time the open / close sequence in software

The sketch here supports both cases:

- normal 3-wire servo: movement test only
- servo with external feedback wire: optional analog-read test

Before using the sketch:

- replace the temporary servo pin with the real cargo-door servo pin once it is confirmed
- if the servo has a feedback wire, set `SERVO_HAS_FEEDBACK_PIN` to `true` and update `FEEDBACK_PIN`
