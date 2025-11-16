#include <painlessMesh.h>
#include <ArduinoJson.h>

#define MESH_PREFIX   "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT     5555

String nodeName = "node4"; // just a label for logs
// uint32_t myId = 3;      //  we use meshId provided by painlessMesh
const uint32_t BAUD = 57600;            // MUST match Pi /dev/ttyAMA0 baud (57600)
const uint16_t MAX_SERIAL_CHUNK = 128;  // keep <= ~200 to avoid fragmentation
const uint32_t HBLOSS = 8000;           // heartbeat loss timeout
bool amIGateway = false;                // Pi-attached node is NOT the gateway
uint32_t lastHB = 0;
uint32_t msgCount = 0;
uint32_t leaderId = 0;

painlessMesh mesh;
Scheduler userScheduler;

const uint32_t DISCOVERY_MS = 7000; // wait for peers/first heartbeat
uint32_t bootAt = 0;

uint32_t meshId = 0;

static const char *HEXCHARS = "0123456789ABCDEF";
String toHex(const uint8_t* data, size_t len) {
  String s; s.reserve(len * 2);
  for (size_t i = 0; i < len; i++) {
    s += HEXCHARS[(data[i] >> 4) & 0xF];
    s += HEXCHARS[data[i] & 0xF];
  }
  return s;
}
size_t fromHex(const String& hex, uint8_t* out, size_t maxlen) {
  size_t n = 0;
  auto v = [](char c)->int {
    if (c >= '0' && c <= '9') return c - '0';
    c &= ~0x20; // upcase
    if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
    return 0;
  };
  for (size_t i = 0; i + 1 < hex.length() && n < maxlen; i += 2) {
    out[n++] = (v(hex[i]) << 4) | v(hex[i+1]);
  }
  return n;
}

void sendMeshMetaToPi(const char *event) {
  DynamicJsonDocument doc(256);
  doc["type"]     = "mesh_meta";
  doc["event"]    = event;        // e.g. "became_leader", "lost_leader", "heartbeat"
  doc["nodeName"] = nodeName;
  doc["meshId"]   = meshId;
  doc["leaderId"] = leaderId;
  doc["uptime"]   = millis();

  String out;
  serializeJson(doc, out);

  // Prefix with @@ so the Pi script can filter these lines easily
  Serial.print("\n@@");
  Serial.println(out);
}

void sendSerialThroughMesh() {
  static uint8_t buff[256];
  int avail = Serial.available();
  if (avail <= 0) return;

  int toRead = min((int)MAX_SERIAL_CHUNK, avail);
  int read = Serial.readBytes(buff, toRead);
  if (read <= 0) return;

  // JSON envelope carrying hex-encoded bytes
  DynamicJsonDocument doc(64 + read * 2);
  doc["msgType"]  = "m";
  doc["hex"]      = toHex(buff, read);
  doc["msgCount"] = msgCount++;
  doc["source"]   = meshId;

  String output;
  serializeJson(doc, output);
  Serial.printf("[tx] UART->mesh sent %d bytes\n", read);

  mesh.sendBroadcast(output);
}

void recieveMessageFromMesh(uint32_t from, String &msg) {
  // Minimal logging to avoid serial slowdown
  // Serial.printf("[%s] rx from %u: %u bytes\n", nodeName.c_str(), from, msg.length());

  DynamicJsonDocument d(2048);
  DeserializationError e = deserializeJson(d, msg);
  if (e) { /* Serial.println("json err"); */ return; }

  const char* type = d["msgType"] | "";
  if (strcmp(type, "m") == 0) {
    const String hex = d["hex"].as<String>();
    static uint8_t buf[512];
    size_t n = fromHex(hex, buf, sizeof(buf));
    if (n > 0) Serial.write(buf, n); // mesh → UART (Pi)
  } else if (strcmp(type, "ctr") == 0) {
    const char* cmd = d["cmd"] | "";
    uint32_t id = d["id"] | 0; 

    if (strcmp(cmd, "heartbeat") == 0) {
      uint32_t hid = id; // sender meshId
      if (leaderId == 0 || hid == leaderId || hid < leaderId) {
        leaderId = hid;
        lastHB   = millis();
        // Serial.printf("leader updated=%u\n", (unsigned)leaderId);
      }
      if (amIGateway && hid < meshId) {
        amIGateway = false;
        leaderId   = hid;
        lastHB     = millis();
        // Serial.println("stepping down; better leader");
      }
    } else if (strcmp(cmd, "whois") == 0) {
      if (amIGateway) {
        DynamicJsonDocument doc(64);
        doc["msgType"] = "ctr";
        doc["cmd"]     = "heartbeat";
        doc["id"]      = meshId;
        String out; serializeJson(doc, out);
        mesh.sendBroadcast(out);
      }
    }
  }
}

void sendHeartBeat() {
  if (!amIGateway) return;
  DynamicJsonDocument doc(64);
  doc["msgType"] = "ctr";
  doc["cmd"]     = "heartbeat";
  doc["id"]      = meshId;
  String msg; serializeJson(doc, msg);
  mesh.sendBroadcast(msg);
  sendMeshMetaToPi("heartbeat");
  lastHB = millis();
}

void electGatewayCheck() {
  if (amIGateway) return;
  if (millis() - bootAt < DISCOVERY_MS) return;

  bool newElection = (millis() - lastHB) > HBLOSS;
  static uint32_t prevLeader = 0;
  static bool     prevRole   = false;

  if (newElection) {
    SimpleList<uint32_t> nodeList = mesh.getNodeList();

    if (nodeList.size() == 0) {
      leaderId   = meshId;
      amIGateway = true;

      DynamicJsonDocument doc(64);
      doc["msgType"] = "ctr";
      doc["cmd"]     = "heartbeat";
      doc["id"]      = meshId;
      String msg; serializeJson(doc, msg);
      mesh.sendBroadcast(msg);
      lastHB = millis();
      // Serial.println("[election] alone -> self-elect");
      return;
    }

    uint32_t minNode = meshId;
    for (auto it = nodeList.begin(); it != nodeList.end(); ++it)
      if (*it < minNode) minNode = *it;

    leaderId   = minNode;
    amIGateway = (leaderId == meshId);

    if (amIGateway) {
      DynamicJsonDocument doc(64);
      doc["msgType"] = "ctr";
      doc["cmd"]     = "heartbeat";
      doc["id"]      = meshId;
      String msg; serializeJson(doc, msg);
      mesh.sendBroadcast(msg);
      lastHB = millis();
    } else {
      lastHB = millis();
      DynamicJsonDocument q(64);
      q["msgType"] = "ctr";
      q["cmd"]     = "whois";
      q["id"]      = meshId;
      String msg; serializeJson(q, msg);
      mesh.sendBroadcast(msg);
    }
  }

  if (leaderId != prevLeader || amIGateway != prevRole) {
    Serial.printf("[%s] leader=%u (me? %s)\n",
      nodeName.c_str(), leaderId, amIGateway ? "yes" : "no");
    prevLeader = leaderId;
    prevRole   = amIGateway;
  }
  // Also tell the Pi whenever its role changes
  if (amIGateway && prevRole == false) {
    sendMeshMetaToPi("became_leader");
  } else if (!amIGateway && prevRole == true) {
    sendMeshMetaToPi("lost_leader");
  }
}

void printStatus() {
  Serial.printf("[%s] meshId=%u leader=%u me? %s lastHB=%lu now=%lu\n",
    nodeName.c_str(), meshId, leaderId, amIGateway ? "yes" : "no",
    (unsigned long)lastHB, (unsigned long)millis());
}

Task statusTask(1000, TASK_FOREVER, &printStatus);
Task sendHeartBeatThroughMesh(1000, TASK_FOREVER, &sendHeartBeat);
Task electionTask(1000, TASK_FOREVER, &electGatewayCheck);

void onConnChange() {
  if (leaderId != 0) lastHB = millis();
  if (!amIGateway && leaderId == 0) {
    SimpleList<uint32_t> nodeList = mesh.getNodeList();
    if (nodeList.size() > 0) {
      uint32_t minNode = meshId;
      for (auto it = nodeList.begin(); it != nodeList.end(); ++it)
        if (*it < minNode) minNode = *it;
      if (minNode != meshId) leaderId = minNode;
    }
  }
}

void setup() {
  Serial.begin(BAUD);
  Serial.setRxBufferSize(256);
  Serial.setTimeout(0);
  delay(200);

  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
  //meshId = mesh.getNodeId();
  Serial.printf("[%s] booted, id=%u\n", nodeName.c_str(), meshId);
  lastHB = millis();
  bootAt = millis();

  mesh.onReceive(&recieveMessageFromMesh);
  mesh.onNewConnection([](uint32_t){ onConnChange(); });
  mesh.onChangedConnections([](){ onConnChange(); });

  userScheduler.addTask(statusTask);               statusTask.enable();
  userScheduler.addTask(sendHeartBeatThroughMesh); sendHeartBeatThroughMesh.enable();
  userScheduler.addTask(electionTask);             electionTask.enable();

  // pump serial often
  Task* serialTask = new Task(10, TASK_FOREVER, &sendSerialThroughMesh);
  userScheduler.addTask(*serialTask);
  serialTask->enable();
}

void loop() {
  mesh.update();
}
