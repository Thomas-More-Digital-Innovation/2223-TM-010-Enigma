# Enigma
CONTEXT
My teacher Jochen Mariën had the assignment of making an enigma machine. The assignment was originally much simpler. An Arduino hooked up to a laptop with some physical buttons to do some settings. At our first meeting I proposed to mimic the enigma as closely as possible in real life with an ESP32 instead of an Arduino. My teacher liked the idea so I started working on it.


Assignment: Make a modern enigma machine.

This was a solo project.

PERSONAL GOALS
With this assignment, I had also set myself some goals such as looking more at the datasheets and trying to program in the ESP-IDF environment. Both ended up working out quite well. Programming in ESP-IDF was quite a challenge that still gave some problems a week before delivery. I then made the choice to switch to Arduino code on the esp32 because otherwise it wasn't going to be finished in time.

HOW IT WORKS
My enigma machine is powered by a 9V battery attached to a switch. When you turn on the switch, 9V reaches an LM7805 which turns it into 5V that can go to the ESP32. Once voltage is applied to the ESP32 it begins a 4 step process.

step 1: setup the rotors
The first step is setting the rotors. There are 3 stepper motors that show purely visually how the rotors are positioned. There are 3 rotary encoders that can set the 3 rotors. By turning a rotary encoder you set the rotor’s starting position. The left rotary encoder for the left rotor,... . There used to be a standard choice of 5 rotors, each rotor was wired differently internally. You had to choose 3 of them and put them into the enigma machine. When you press on a rotary encoder you chose 1 of the 5 rotors in the code. The old enigma machines let you know the uncoded letter by lighting a light. My solution to this was a ws2812b led strip. While setting the rotors, the led strip indicates which rotor you choose when pressing and which letter you will start on when turning. When the rotors are set to your choice you can press start message. When start message is pressed the code begins step 2: reading the switchboard.

step 2: read the switchboard
The switchboard requires 26 GPIO pins. In each case, 1 pin is set as high output and all other pins are read as inputs. Then if 1 of those inputs is high it means they are paired and are linked together in the code. After reading the switchboard, step 3 begins: reading the keyboard.

step 3: read the keyboard
The keyboard is a button matrix so I only need 11 GPIO pins, 4 outputs (rows) and 7 inputs (columns). By setting 1 output high each time and checking all inputs, I know by the combination of row and column which button was pressed. Each letter you press goes in the code through the switchboard, the 3 rotors, the reflector, again through the 3 rotors and finally once more through the switchboard. The encoded letter that comes out of that will light up at the LED strip. Each time a key is pressed, the right rotor turns one letter. This way if you press the same letter twice you will never get the same encoded letter back. When you have finished typing your message you can press stop message, then step 4 starts: send message if needed.

step 4: send message if needed
When you press stop message the status of the WiFi switch is checked. If it is off then the message is not sent and you go back to step 1: setting up the rotors. If the switch is on the ESP32 connects to a WiFi network. Then the message is forwarded with a POST request so it can be read on a website. The settings of the rotors are then sent via MQTT to a NodeRed server running in the IBM cloud. In NodeRed that message is then sent to me via Whatsapp. After sending, you can get back to step 1.

FRAMING
For the framing, I used the laser cutter and the 3D printer we have available at school. Everything for the laser cutter was drawn in AutoCAD and the keys and rotors were drawn in fusion360. I had worked with a laser cutter before but it was the first time for me to 3D print. For the box around the enigma, I got a lot of help from my grandfather. He used to be a carpenter and helped me make the wooden wine tray into the beautiful frame that it is now.

MY CONTRIBUTION
solo project
got help with the wood assembly
CONCLUSION
I thought it was a very interesting project in which I was able to do a lot that I said I wanted to try at the end of last school year, such as 3D printing and working with ESP-IDF. I'm also glad I got another chance to design a PCB.
