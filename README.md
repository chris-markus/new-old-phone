# Rotary Phone Prop Conversion

A quick hack for a friend to convert a rotary phone to ring at intervals and play an audio track through the handset when it is picked up.

> **Important:** This phone is no longer compatible with a standard phone line. Do not plug it in to a wall jack and do not plug the interface box into any other phone!

## Operation

When powered, the phone will wait the amount of time dictated the interface box knob (1min - 10min) and ring when the time has elapsed. The phone will ring 5 times. If it is picked up while it is ringing, it will play the next audio file in line on the SD card. When the reciever is replaced, it will stop audio playback and begin waiting to ring again. If the phone is not answered, it will likewise reset and begin waiting again.

*Note that the phone will not ring if it is not on the hook.*

## Adding Audio Files

1. Ensure the phone is disconnected from power.
1. Locate and remove the microSD card from the underside of the phone.
1. Copy the desired number of audio files onto the SD card. The phone will rotate through them each successive time it rings. Supported audio formats are: mp3, WAV, WMA at up to 48kHz.
1. Replace the SD card and then reconnect power.

## "Ring Now"

Pressing the "Ring" button on the interface box will cause the phone to ring now if it is not currently ringing or playing back audio. If the phone is not on the hook it will skip the ring and immediately play the next audio file. After the phone is replaced on the hook, it will reset its wait timer and begin waiting to ring again.

## Techical Details

### Hardware

TODO: schematic

Broadly the hardware is in two parts:

- Within the phone there's an Arduino Nano, MP3 player dev board, H-bridge dev board, and a buck converter module. These are connected via a phone cable to an interface box which provides 24V power, a button contact and a potentiometer voltage output.
  - 0-24V potentiometer output is divided by a 10k/69k voltage divider. 24V on the input -> `24V * 10k / (10k + 69k) = 3.038V` at the microcontroller. Microcontroller is safe up to `5V * (10k + 69k) / 10k = 39.5V`, well above any transients on a 24V line.
- The interface box makes use of the 4 conductors in the phone cable as:
  - Red: 24V Power.
  - Black: Ground.
  - Green: 0-24V potentiometer voltage output (for controlling ring delay).
  - Yellow: Button line (normally-open; floating/grounded).

#### Parts

Dev boards and modules to the rescue!

1. 1x Arduino Nano
1. 1x Buck converter module (https://www.amazon.com/dp/B01MQGMOKI?ref=ppx_yo2ov_dt_b_fed_asin_title&th=1)
    - Strongly recommend a switching converter like these over a linear regulator; 24V->5V on a linear regulator is quite a bit of heat dissipation.
1. 1x L298N H-Bridge motor driver (https://www.amazon.com/dp/B0C5JCF5RS?ref=ppx_yo2ov_dt_b_fed_asin_title)
1. 1x MP3 Player module (https://www.amazon.com/dp/B0CF593SWY?ref=ppx_yo2ov_dt_b_fed_asin_title)
    - These seem to use a GD3300B chip or equivalent.
1. 1x 24V DC power supply (https://www.amazon.com/dp/B09281KTS8?ref=ppx_yo2ov_dt_b_fed_asin_title)
    - 1A is more than enough.
1. 1x Phone cord and recepticle (for interface box) (https://www.amazon.com/dp/B01MQDG61S?ref=ppx_yo2ov_dt_b_fed_asin_title)
1. Button and potentiometer (any button will do, suggest 10kOhm+ potentiometer).
1. Assorted hookup wire, headers, etc.

### Software

Software is a simple Arduino sketch. Code [here](./new-old-phone/new-old-phone.ino).

Notables:

- 60Hz ISR to create bell driver waveform. 3 iterations per drive period -> 20Hz bell ring.
  - My specific phone needed some tuning to get the coil to drive the bell with only +/-24V; 2 of the 3 ISR calls drive "forward" 1/3 drives "backward" with the 60Hz ISR gave a reliable ring. ("Forward" and "backward" will depend on your coil orientation. I'm using "forward" here to mean away from the bell and against the spring).
- A global state machine to handle timing made the code easy to write and less bug-prone than other approaches.

## Remaining Work

The *ideal* version of this project would have not required any modification to the phone itself and instead created an interface box which interfaced with the phone's stock electronics. The code could easily be modified to control such a box. Hardware would need to be largely redesigned. If I have some time I'll come back and add a new hardware option to do that.
