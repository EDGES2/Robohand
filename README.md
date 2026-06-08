# Robo Hand

A robotic hand controlled in real time by hand gestures captured via webcam. Built entirely from scratch — including a custom-etched PCB, cardboard frame, and fishing-line mechanics.

<img width="720" height="405" alt="hand_and_screen__video" src="https://github.com/user-attachments/assets/896bdb9d-cc4f-4bc6-83ce-5b51cac2294e" />


---

## How It Works

A Python script captures webcam frames and uses **MediaPipe** to detect 21 hand landmarks in real time. Finger bend ratios are calculated in palm-local 3D coordinates, mapped to discrete servo stages, and sent as `<IDX,ANGLE>` packets over USB Serial to an **Arduino Nano**. The Arduino drives five **SG90 180° servos** that pull fishing line to curl each finger.

```
Webcam → MediaPipe landmarks → bend ratio → stage angle → Serial → Arduino → SG90 servo → finger
```

---

## Build Process

The project went through multiple full iterations — from rough prototypes to working hardware.

### PCB — 10 attempts, hand-etched

Every board was designed, printed on glossy paper with a laser printer, transferred with a clothes iron, and etched in a hydrogen peroxide + salt + citric acid solution. The photo below shows the full progression from the first failed attempt to the final working board.

<!-- Photo 1: all PCB attempts laid out in a row -->
<img width="3998" height="456" alt="PCBs" src="https://github.com/user-attachments/assets/fbd966dc-3b0c-410d-8f40-766d3ab15fc3" />

The servo power supply board takes an **18650 Li-ion cell**, steps the voltage up to 5–6 V via a DC-DC converter, and delivers it to the servos. The Arduino Nano is powered separately over USB from the laptop

<!-- Photo 3: finished PCB with Arduino Nano, step-up module, and 18650 battery connected -->
<img width="2540" height="1440" alt="circuit" src="https://github.com/user-attachments/assets/956e02b8-ee4b-4021-8e09-b8219fe67f26" />


### Mechanical Hand — cardboard frame + latex glove

The frame is made from cardboard cut with a knife, assembled using spaghetti sticks, later replaced with toothpicks. The hand itself is also cut from cardboard, with small pieces of plastic drinking straw used as fishing line guides. Fingers curl by pulling the fishing line taut. A latex glove is stretched over the top for a cleaner look.

<!-- Photo 2: robotic hand with latex glove on cardboard base (and the cat stickers) -->


<p align="center">
  <img height="500" alt="hand_front" src="https://github.com/user-attachments/assets/6709a5a1-8f8e-4246-9391-4531f75c73f8" />
  <img height="500" alt="hand_back" src="https://github.com/user-attachments/assets/270e5a4c-e2d1-48c5-a64e-64aa622f2be5" />
  <img height="500" alt="hand_front_without_glove" src="https://github.com/user-attachments/assets/fc54f19b-edd9-408c-af19-4e8f09ecc568" />
</p>



Each finger is connected to its servo by a single strand of fishing line routed through the straws. Pulling the line curls the finger, releasing it allows the glove's elasticity to extend it.

### Software

- **MediaPipe Hand Landmarker** (`hand_landmarker.task`) — detects 21 3D landmarks per hand
- **Palm-local coordinate system** — bend ratios are computed relative to the palm plane, making detection rotation-invariant
- **Stage-based servo control** — instead of continuous angles, each finger snaps between 4 discrete positions to reduce jitter and servo stress
- **Dead zone filter** — ignores micro-fluctuations below 15% bend change
- **Auto timeout** — if Serial connection drops, all fingers return to open position

<!-- GIF 2: robotic hand only, curling and extending fingers -->
<img width="360" height="640" alt="hand_video" src="https://github.com/user-attachments/assets/c57d1205-3f5d-461d-97cd-e66c72922b5c" />




---

## Servo Wiring

| Finger     | Arduino Pin |
|------------|-------------|
| Thumb (0)  | D2          |
| Index (1)  | D4          |
| Middle (2) | D6          |
| Ring (3)   | D7          |
| Pinky (4)  | D11         |

---

## Serial Protocol

Commands are sent as ASCII packets:

```
<IDX,ANGLE>
```

| Field   | Description              |
|---------|--------------------------|
| `IDX`   | Finger index, 0–4        |
| `ANGLE` | Target angle, 0–180°     |

Example: `<1,80>` — curl index finger to 80°.

---

## Setup

**Dependencies:**
```bash
pip install opencv-python mediapipe pyserial
```

**Run:**
```bash
python hand_tracking.py
```

The script auto-detects the Arduino's COM port. To set it manually, edit `SERIAL_PORT` in `hand_tracking.py`.

Press `Q` to quit. On exit, all fingers return to the open position.

---

## Built With

- [MediaPipe](https://ai.google.dev/edge/mediapipe/solutions/vision/hand_landmarker) — hand landmark detection
- [OpenCV](https://opencv.org/) — webcam capture and UI overlay
- [Arduino](https://www.arduino.cc/) — servo control (Arduino Nano)
- [pyserial](https://pyserial.readthedocs.io/) — USB Serial communication
- SG90 180° servos × 5
- 18650 Li-ion cell + DC-DC step-up converter
- Hand-etched PCB (laser toner transfer + peroxide/salt/citric acid etching)
