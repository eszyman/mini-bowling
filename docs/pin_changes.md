# Summary of pinout changes
The the number of changes needed from the original script was minimized, however some pin out changes were required on the Arduino Mega board. I have made note for my own personal record on what has been changed here.

| Variable | Original Arduino Assignment | New Arduino Assignment | Reason for change |
| :--- | :--- | :--- | :--- |
| `STEPPER_ENABLE_PIN` | 22 | 49 | Pin 22 can be used here, however I had already used pin 49. |
| `BALL_SENSOR_PIN` | A0 | 2 | Makes ready hardware interrupt pin for faster ball detection. |
| `BALL_SPEED_PIN` | A1 | 3 | Makes ready hardware interrupt pin for faster ball detection. |
| `STEP_PIN` | 2 | A0 | Pin 2/3 required for possible HW interrupt. |
| `DIR_PIN` | 3 | A1 | Pin 2/3 required for possible HW interrupt. |
| `PINSETTER_RESET_PIN` |41 | 48 | I just decided to use this pin to add the user button |

### Major Changes required for ESP -> WLED Control
| Variable | Original Arduino Assignment | New Arduino Assignment | Reason for change |
| :--- | :--- | :--- | :--- |
| `SM_PIN_9` | 18 | 21 | Pin 18 was required for `Serial1` output to WLED. |
| `No Assignment Needed` | N/A | 18 | Pin 18 is required for `Serial1` output to WLED. |

### Summary of all pinouts
...to do
