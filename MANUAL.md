A fuzzy Logic Temperature controller for diy brewing systems (BIAB, RIMS ...)
Simple, but effective. No need for PID !

### Control Logic
The system uses fuzzy logic based on 4 rules:

1. **Idle Zone** (Δ < -0.1°C)
   - Power = 0%
   - Prevents temperature overshoot

2. **Setpoint Control Zone** (-0.1°C < Δ < 0.1°C)
   - Power = Maintenance Power + Ajustable Offset
   - Uses computed logarithmic curve for stable temperature
   - Manual fine-tuning available

3. **Ramp Zone** (0.1°C < Δ < 1.5°C)
   - Power increases smoothly to maximum + Ajustable Offset
   - Provides controlled temperature rise

4. **Full Power Zone** (Δ > 1.5°C)
   - Power = Maximum (90%) + Ajustable Offset
   - Rapid temperature increase

Where:
- Δ = Target Temperature - Current Temperature
- All thresholds are customizable in settings

### User Interface
- Intuitive rotary encoder navigation
- Clear contextual menus
- Led indicator for heating (SSR indicator)
- Sound alerts when setpoint reached and stabilized
- Auto Timer start when in setpoint zone 
- Parameter saving in EEPROM

## User Guide

### Basic Navigation
- **Rotation**: Adjust values
- **Single Click**: Confirm/Select
- **Long push**: Secondary menu/Fine adjustment
- **Very Long push**: Return to main menu

### Main Modes

#### 1. Manual Mode
- Displays current and target temperatures
- Direct temperature setpoint adjustment
- Fine mode (F) for precise power adjustment : offset tweaking

#### 2. Settings
Control parameter adjustment:
- Temperature offset
- PWM period
- EMA filtering
- Fuzzy control zones
- Power limits

#### 3. Memory
- Save parameters
- Load parameters
- Factory reset


## Required Hardware
- Arduino Nano (recommended for size and pin layout)
- Universal Proto Shield PCB for Arduino Nano
- 20x4 I2C LCD Display
- TM1637 Display
- Rotary Encoder with button
- DS18B20 Waterproof Temperature Probe
- SSR Relay (40A recommended)
- WS2812 RGB LED
- Buzzer
- Dupont cables (Male-Male, Male-Female)
- Terminal blocks for 220V connections
- Heat shrink tubes for connections

## Assembly Recommendations
- Use Arduino Nano Proto Shield as main board
- Mount all low voltage components on the shield
- Use Dupont connectors for easy maintenance
- Separate high voltage components in dedicated enclosure
- Use terminal blocks for SSR connections
- Heat shrink all solder joints
- Label all connections
- Add strain relief for probe cable

## Wiring

### Digital Pins
- D2: Encoder A (Interrupt)
- D3: Encoder B (Interrupt)
- D4: Encoder Button
- D5: TM1637 CLK
- D6: TM1637 DIO
- D7: DS18B20 Data (OneWire)
- D9: SSR Control
- D10: Buzzer
- D11: WS2812 LED

### I2C Connection (LCD)
- A4: SDA
- A5: SCL

### Power
- SSR: Connect to 220V with proper isolation
- DS18B20: Requires 4.7kΩ pull-up resistor between data and VCC
- All components except SSR powered by Arduino 5V and GND

### Notes
- Use shielded cables for temperature probe
- Keep high voltage (SSR) and low voltage wiring separated
- Ground all metal parts for safety

## Dependencies
Required Arduino libraries:
- OneWire
- DallasTemperature
- TM1637Display
- Adafruit_NeoPixel
- Wire (built-in)
- EEPROM (built-in)

## Building and Installation
1. Install PlatformIO
2. Clone repository
3. Connect Arduino
4. Build & upload code

## DS18B20 Draft Calibration 
1. Fill system with some water
2. Set temperature to about 60°C
3. Once stable, measure actual temperature with calibrated thermometer
4. Adjust probe offset in settings
5. Test multiple temperature points


## Using Calibration Script for power curve

### Prerequisites on computer
1. Install Python and required packages:

pip install numpy scipy matplotlib


### Collecting Data Points
1. On your brewing system:
   - Start with full water volume (typical brew volume + estimated grain volume)
   - Enter Brew mode and enable Fine Adjustment (press and hold encoder, and release)
   - For each temperature (40°C to 90°C):
     a. Set target temperature and wait for stabilization (≈15min)
     b. Fine tune PWM until temperature is perfectly stable
     c. Write down the temperature and PWM % values
   - Recommended test points: 40°C, 50°C, 60°C, 70°C, 80°C, 90°C

### Running Calibration
1. Run the script:
python calibrate.py


2. The script will:
   - Calculate optimal A, B, C coefficients
   - Show a graph of your measurements vs calculated curve


### Using the Results
1. In your controller's Settings menu:
   - Go to Power Curve section
   - Enter the A, B, C values exactly as shown
   - Values typically range:
     * A: 0.01 to 0.02
     * B: 0.08 to 0.09
     * C: 6 to 7

### Verification
1. Test several temperatures
2. PWM should stabilize automatically
3. Fine adjustment should require minimal offset tweaking (±1%)

### Troubleshooting
- If curve doesn't fit well :
  * Start again, and take more measurement points
  * Ensure system was stable at each point
  * Check for external factors (drafts, lid position)
- If power seems too high/low:
  * Verify your volume matches calibration volume
  * Check thermal insulation
  * Consider ambient temperature effects