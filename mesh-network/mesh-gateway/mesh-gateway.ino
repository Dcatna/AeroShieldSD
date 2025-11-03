#include <painlessMesh.h>
#include <ArduinoJson.h>

#define MESH_PREFIX   "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT     5555

String nodeName = "node1"; //change to whatever the node is (doesnt matter just make it different from the others)
const uint32_t BAUD = 115200;           // match Pixhawk
const uint16_t MAX_SERIAL_CHUNK = 128; //max bytes we can read at a time
const uint32_t HBLOSS = 4000; //time without heartbeat before switch 
bool amIGateway = true; //first node will be gateway

painlessMesh mesh;
Scheduler userScheduler;

uint32_t msgCount = 0;
uint32_t nodeId = 0; //used for gateway, change with each node

static const char *HEX = "0123456789ABCDEF";
String toHex(const uint8_t* data, size_t len){ //just using this to make it readable for us
  String s; s.reserve(len*2);
  for(size_t i=0;i<len;i++){ s += HEX[(data[i]>>4)&0xF]; s += HEX[data[i]&0xF]; }
  return s;
}
size_t fromHex(const String& hex, uint8_t* out, size_t maxlen){
  size_t n = 0;
  for(size_t i=0; i+1<hex.length() && n<maxlen; i+=2){
    char h=hex[i], l=hex[i+1];
    auto v = [](char c)->int{
      if(c>='0'&&c<='9') return c-'0';
      c &= ~0x20; // upper
      if(c>='A'&&c<='F') return 10 + (c-'A');
      return 0;
    };
    out[n++] = (v(h)<<4) | v(l);
  }
  return n;
}

void sendSerialThroughMesh() {
  uint8_t buff[256];
  if (Serial.available()) {
    int read = Serial.readBytes(buff, min((int)MAX_SERIAL_CHUNK, Serial.available())); //we chose the min here so either the max we can read or what is available from serial
    if (read == 0) {
      break;
    }
    DynamicJsonDocument doc(64 + rd*2); //we use this size bc when each byte is converted to hex it becomes 2 characters (bytes read*2) then add 64 for the key names and anything else
    doc["hex"] = toHex(buff, read); //we send this and not the binary bc painlessmesh with esp8266 only supports strings and not raw bytes, so we just encode and decode each message
    doc["source"] = nodeId;
    doc["msgCount"] = msgCount++;
    doc["msgType"] = "m"; //in case we want to clarify the types of messages sent later?

    string output;
    serializeJson(doc, output);
    mesh.sendBroadcast(output);
  }
}

void recieveMessageFromMesh(uint32_t from, String &msg) { //recieve message from mesh and write it to the pixhawk
  DynamicJsonDocument d(2048);
  DeserializationError e = deserializeJson(d, msg);
  if(e) return;

  if (strcmp(d["msgType"], "m") == 0) {
    const String hex = d["hex"].as<String>();
    static uint8_t buf[512];
    size_t n = fromHex(hex, buf, sizeof(buf));
    if(n>0) Serial.write(buf, n); //this is sending the serial to the pixhawk to run the commands
  } // add more message cases later
}

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  mesh.update();
}
