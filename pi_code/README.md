# pi_code Folder Structure
This conatins the code used on the PI to test the mesh network and the object detection

## Directrory Structure
```
pi_code
| - bit_err_test.py -- used for sending large amounts of data, increasing the payload to check for bit errors
| - bs_rec.py -- used on the basestation side to receive and decode data being sent from the leader node
| - calibrate.json -- calibration for YOLO model
| - esp_rec.py -- ran on all of the PIs to check leader status, collect messages, and forward to the base station
| - esp_sender.py -- ran on all of the PIs, to run the YOLO model and send structued packets throughout the network
| - esp_sender_wmav.py -- ran on all of the PIs to collect and send MAVLink (telementry) information throughout the network
| - reqs.txt -- venv requirements file
| - uart_flood.py -- used to see the maximum amount of messages the ESP 8266 can handle sending
| - yolov8n.pt

```

## Running The System
1. On each PI set up your virutal environment
   - ` python -m venv venv`
2. Install reqs.txt
   - `pip install -r reqs.txt`
3. Run both esp_sender.py and esp_rec.py
4. On basestation run bs_rec.py
