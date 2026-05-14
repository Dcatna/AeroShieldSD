#include <painlessMesh.h>
#include <ArduinoJson.h>

#define MESH_PREFIX   "whateverYouLike"
#define MESH_PASSWORD "somethingSneaky"
#define MESH_PORT     5555

String nodeName = "node3";
uint32_t myId = 3;                  // logical node ID used in your app heartbeat JSON
uint32_t meshId = 3;                // logical app-level node ID
uint32_t myMeshNodeId = 0;          // actual painlessMesh node ID for this ESP
uint32_t leaderMeshNodeId = 0;      // actual painlessMesh node ID of elected leader

const uint32_t BAUD = 57600;
const uint32_t HBLOSS = 8000;
const uint32_t DISCOVERY_MS = 7000;

bool amIGateway = false;
uint32_t lastHB = 0;
uint32_t msgCount = 0;
uint32_t leaderId = 2;              // logical leader ID from heartbeat JSON

uint32_t rx_msgs = 0;
uint32_t rx_wire_bytes = 0;
uint32_t rx_payload_bytes = 0;

uint32_t tx_msgs = 0;
uint32_t tx_wire_bytes = 0;
uint32_t tx_payload_bytes = 0;

uint32_t lastRateMs = 0;
uint32_t lastHB_painless = 0;
uint32_t bootAt = 0;
uint32_t lastPrintedLeader = 0;
uint32_t lastRxPrintMs = 0;

painlessMesh mesh;
Scheduler userScheduler;

static const uint8_t SYNC1 = 0xAA;
static const uint8_t SYNC2 = 0x55;
static const uint16_t MAX_PAYLOAD = 256;
static const size_t MAX_FRAME = 2 + 2 + 2 + MAX_PAYLOAD + 1;
static const size_t MAX_B64_LEN = ((MAX_FRAME + 2) / 3) * 4;
static const uint32_t RX_PRINT_INTERVAL_MS = 250;

uint8_t checksum8(const uint8_t* data, size_t len) {
  uint32_t sum = 0;
  for (size_t i = 0; i < len; i++) sum += data[i];
  return (uint8_t)(sum & 0xFF);
}

static const char B64_TABLE[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

size_t b64EncodedLen(size_t inputLen) {
  return ((inputLen + 2) / 3) * 4;
}

int b64Index(char c) {
  if (c >= 'A' && c <= 'Z') return c - 'A';
  if (c >= 'a' && c <= 'z') return c - 'a' + 26;
  if (c >= '0' && c <= '9') return c - '0' + 52;
  if (c == '+') return 62;
  if (c == '/') return 63;
  return -1;
}

bool base64Encode(const uint8_t* input, size_t inputLen, String& out) {
  size_t outLen = b64EncodedLen(inputLen);
  if (!out.reserve(outLen + 2)) return false; // +2 for "D:"
  out = "D:";

  for (size_t i = 0; i < inputLen; i += 3) {
    uint32_t octet_a = input[i];
    uint32_t octet_b = (i + 1 < inputLen) ? input[i + 1] : 0;
    uint32_t octet_c = (i + 2 < inputLen) ? input[i + 2] : 0;

    uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

    out += B64_TABLE[(triple >> 18) & 0x3F];
    out += B64_TABLE[(triple >> 12) & 0x3F];
    out += (i + 1 < inputLen) ? B64_TABLE[(triple >> 6) & 0x3F] : '=';
    out += (i + 2 < inputLen) ? B64_TABLE[triple & 0x3F] : '=';
  }

  return true;
}

bool base64Decode(const char* input, size_t inputLen, uint8_t* output, size_t& outputLen) {
  if (inputLen % 4 != 0) return false;

  size_t padding = 0;
  if (inputLen >= 1 && input[inputLen - 1] == '=') padding++;
  if (inputLen >= 2 && input[inputLen - 2] == '=') padding++;

  size_t decodedLen = (inputLen / 4) * 3 - padding;
  if (decodedLen > MAX_FRAME) return false;

  size_t outIdx = 0;
  for (size_t i = 0; i < inputLen; i += 4) {
    int a = b64Index(input[i]);
    int b = b64Index(input[i + 1]);
    int c = (input[i + 2] == '=') ? 0 : b64Index(input[i + 2]);
    int d = (input[i + 3] == '=') ? 0 : b64Index(input[i + 3]);

    if (a < 0 || b < 0 || (input[i + 2] != '=' && c < 0) || (input[i + 3] != '=' && d < 0)) {
      return false;
    }

    uint32_t triple = ((uint32_t)a << 18) | ((uint32_t)b << 12) | ((uint32_t)c << 6) | (uint32_t)d;

    if (outIdx < decodedLen) output[outIdx++] = (triple >> 16) & 0xFF;
    if (outIdx < decodedLen) output[outIdx++] = (triple >> 8) & 0xFF;
    if (outIdx < decodedLen) output[outIdx++] = triple & 0xFF;
  }

  outputLen = decodedLen;
  return true;
}

bool sendFrameToMeshNode(uint32_t targetMeshNodeId, const uint8_t* frame, size_t frameLen, size_t payloadLen) {
  if (targetMeshNodeId == 0 || frame == NULL || frameLen == 0) return false;

  String msg;
  if (!base64Encode(frame, frameLen, msg)) {
    Serial.println("TX encode failed");
    return false;
  }

  bool ok = mesh.sendSingle(targetMeshNodeId, msg);

  if (ok) {
    tx_msgs++;
    tx_wire_bytes += frameLen;
    tx_payload_bytes += payloadLen;

    Serial.printf("TX frame: targetMesh=%u frame=%u payload=%u b64=%u\n",
                  targetMeshNodeId,
                  (unsigned)frameLen,
                  (unsigned)payloadLen,
                  (unsigned)msg.length());
  } else {
    Serial.printf("TX send failed: targetMesh=%u frame=%u\n",
                  targetMeshNodeId,
                  (unsigned)frameLen);
  }

  return ok;
}

void sendMeshMetaToPi(const char *event) {
  DynamicJsonDocument doc(256);
  doc["type"] = "mesh_meta";
  doc["event"] = event;
  doc["nodeName"] = nodeName;
  doc["meshId"] = meshId;
  doc["leaderId"] = leaderId;
  doc["leaderMeshNodeId"] = leaderMeshNodeId;
  doc["myMeshNodeId"] = myMeshNodeId;
  doc["amIGateway"] = amIGateway;
  doc["uptime"] = millis();

  String out;
  serializeJson(doc, out);

  Serial.print("\n@@");
  Serial.println(out);
}

void printThroughput() {
  uint32_t now = millis();
  if (lastRateMs == 0) lastRateMs = now;

  uint32_t dt_ms = now - lastRateMs;
  if (dt_ms == 0) return;

  rx_msgs = rx_wire_bytes = rx_payload_bytes = 0;
  tx_msgs = tx_wire_bytes = tx_payload_bytes = 0;
  lastRateMs = now;
}

void sendLeaderStatusToPi(const char* event, bool wasLeader) {
  DynamicJsonDocument doc(256);
  doc["type"] = "leader_status";
  doc["event"] = event;
  doc["nodeName"] = nodeName;
  doc["meshId"] = meshId;
  doc["myMeshNodeId"] = myMeshNodeId;
  doc["leaderId"] = leaderId;
  doc["leaderMeshNodeId"] = leaderMeshNodeId;
  doc["wasLeader"] = wasLeader;
  doc["isLeader"] = amIGateway;
  doc["uptime"] = millis();

  String out;
  serializeJson(doc, out);

  Serial.print("\n@@");
  Serial.println(out);
}

void setLeaderState(bool newIsLeader, const char* event) {
  bool wasLeader = amIGateway;
  amIGateway = newIsLeader;

  if (wasLeader != amIGateway) {
    sendLeaderStatusToPi(event, wasLeader);
  }
}

void readFramedSerialAndHandleData() {
  static enum {
    WAIT_SYNC1,
    WAIT_SYNC2,
    READ_LEN1,
    READ_LEN2,
    READ_SEQ1,
    READ_SEQ2,
    READ_PAYLOAD,
    READ_CRC
  } state = WAIT_SYNC1;

  static uint16_t payloadLen = 0;
  static uint16_t seq = 0;
  static uint16_t idx = 0;
  static uint8_t payload[MAX_PAYLOAD];
  static uint8_t crcRx = 0;
  static uint32_t lastByteMs = 0;

  const uint32_t FRAME_TIMEOUT_MS = 200;

  if (state != WAIT_SYNC1 && (millis() - lastByteMs > FRAME_TIMEOUT_MS)) {
    Serial.println("UART frame timeout, resetting parser");
    state = WAIT_SYNC1;
    payloadLen = 0;
    seq = 0;
    idx = 0;
  }

  while (Serial.available() > 0) {
    uint8_t b = (uint8_t)Serial.read();
    lastByteMs = millis();

    switch (state) {
      case WAIT_SYNC1:
        if (b == SYNC1) state = WAIT_SYNC2;
        break;

      case WAIT_SYNC2:
        if (b == SYNC2) state = READ_LEN1;
        else state = WAIT_SYNC1;
        break;

      case READ_LEN1:
        payloadLen = b;
        state = READ_LEN2;
        break;

      case READ_LEN2:
        payloadLen |= ((uint16_t)b << 8);
        if (payloadLen == 0 || payloadLen > MAX_PAYLOAD) {
          Serial.printf("UART invalid payload len=%u\n", (unsigned)payloadLen);
          state = WAIT_SYNC1;
        } else {
          state = READ_SEQ1;
        }
        break;

      case READ_SEQ1:
        seq = b;
        state = READ_SEQ2;
        break;

      case READ_SEQ2:
        seq |= ((uint16_t)b << 8);
        idx = 0;
        state = READ_PAYLOAD;
        break;

      case READ_PAYLOAD:
        payload[idx++] = b;
        if (idx >= payloadLen) state = READ_CRC;
        break;

      case READ_CRC: {
        crcRx = b;

        uint8_t hdr[4];
        hdr[0] = payloadLen & 0xFF;
        hdr[1] = (payloadLen >> 8) & 0xFF;
        hdr[2] = seq & 0xFF;
        hdr[3] = (seq >> 8) & 0xFF;

        uint8_t crcCalc = (checksum8(hdr, 4) + checksum8(payload, payloadLen)) & 0xFF;

        if (crcCalc == crcRx) {
          uint8_t frame[MAX_FRAME];
          size_t frameLen = 0;

          frame[frameLen++] = SYNC1;
          frame[frameLen++] = SYNC2;
          frame[frameLen++] = hdr[0];
          frame[frameLen++] = hdr[1];
          frame[frameLen++] = hdr[2];
          frame[frameLen++] = hdr[3];
          memcpy(&frame[frameLen], payload, payloadLen);
          frameLen += payloadLen;
          frame[frameLen++] = crcRx;

          if (amIGateway) {
            // Leader sends its own locally received UART data back to the Pi
            Serial.write(frame, frameLen);
            Serial.printf("Leader forwarded local UART frame to Pi: frame=%u payload=%u seq=%u\n",
                          (unsigned)frameLen,
                          (unsigned)payloadLen,
                          (unsigned)seq);
          } else {
            // Non-leader forwards local UART data to leader over mesh
            if (leaderMeshNodeId != 0) {
              sendFrameToMeshNode(leaderMeshNodeId, frame, frameLen, payloadLen);
            } else {
              Serial.printf("No leader mesh node known, dropping UART frame len=%u seq=%u\n",
                            (unsigned)payloadLen,
                            (unsigned)seq);
            }
          }
        } else {
          Serial.printf("UART CRC mismatch calc=%u rx=%u len=%u seq=%u\n",
                        crcCalc, crcRx, (unsigned)payloadLen, (unsigned)seq);
        }

        state = WAIT_SYNC1;
        break;
      }
    }
  }
}

void recieveMessageFromMesh(uint32_t from, String &msg) {
  Serial.printf("[node%u] rx from meshNode %u: %u bytes\n",
                meshId, from, (unsigned)msg.length());

  if (msg.startsWith("D:")) {
    const char* b64 = msg.c_str() + 2;
    size_t b64Len = msg.length() - 2;
    uint8_t frame[MAX_FRAME];
    size_t frameLen = 0;

    if (!base64Decode(b64, b64Len, frame, frameLen)) {
      Serial.printf("RX decode failed from=%u b64=%u\n", from, (unsigned)b64Len);
      return;
    }

    if (frameLen < 7 || frame[0] != SYNC1 || frame[1] != SYNC2) {
      Serial.printf("RX decoded frame invalid header from=%u frame=%u\n",
                    from, (unsigned)frameLen);
      return;
    }

    uint16_t payloadLen = (uint8_t)frame[2] | ((uint16_t)frame[3] << 8);
    size_t expectedLen = 2 + 2 + 2 + payloadLen + 1;

    if (payloadLen > MAX_PAYLOAD || frameLen != expectedLen) {
      Serial.printf("RX decoded frame len mismatch from=%u frame=%u expected=%u payload=%u\n",
                    from, (unsigned)frameLen, (unsigned)expectedLen, (unsigned)payloadLen);
      return;
    }

    const uint8_t* payload = &frame[6];
    uint8_t crcRx = frame[frameLen - 1];

    uint8_t hdr[4];
    hdr[0] = frame[2];
    hdr[1] = frame[3];
    hdr[2] = frame[4];
    hdr[3] = frame[5];

    uint8_t crcCalc = (checksum8(hdr, 4) + checksum8(payload, payloadLen)) & 0xFF;
    if (crcCalc != crcRx) {
      Serial.printf("RX decoded CRC mismatch from=%u calc=%u rx=%u\n", from, crcCalc, crcRx);
      return;
    }

    rx_msgs++;
    rx_wire_bytes += frameLen;
    rx_payload_bytes += payloadLen;

    if (millis() - lastRxPrintMs >= RX_PRINT_INTERVAL_MS) {
      Serial.printf("RX frame OK: from=%u frame=%u payload=%u b64=%u\n",
                    from,
                    (unsigned)frameLen,
                    (unsigned)payloadLen,
                    (unsigned)msg.length());
      lastRxPrintMs = millis();
    }

    if (amIGateway) {
      Serial.write(frame, frameLen);
      Serial.printf("Leader forwarded mesh frame from %u to Pi: frame=%u payload=%u\n",
                    from,
                    (unsigned)frameLen,
                    (unsigned)payloadLen);
    } else {
      Serial.printf("Non-leader received data frame from %u, ignoring UART forward\n", from);
    }

    return;
  }

  DynamicJsonDocument d(256);
  DeserializationError e = deserializeJson(d, msg);
  if (e) {
    Serial.printf("RX non-data/non-json from=%u len=%u\n", from, (unsigned)msg.length());
    return;
  }

  const char* type = d["msgType"] | "";

  if (strcmp(type, "ctr") == 0) {
    const char* cmd = d["cmd"] | "";
    uint32_t id = d["id"] | 0;

    if (strcmp(cmd, "heartbeat") == 0) {
      uint32_t hid = id;

      Serial.printf("Heartbeat from logical=%u mesh=%u\n", hid, from);

      // Lower logical meshId always wins
      if (hid < meshId) {
        if (amIGateway || leaderId != hid || leaderMeshNodeId != from) {
          leaderId = hid;
          leaderMeshNodeId = from;
          lastHB = millis();

          setLeaderState(false, "lost_leader");

          Serial.printf("Following lower leader logical=%u mesh=%u\n", hid, from);

          if (leaderId != lastPrintedLeader) {
            Serial.printf("Leader = %u (mesh node %u)\n", leaderId, leaderMeshNodeId);
            lastPrintedLeader = leaderId;
          }
        } else {
          lastHB = millis();
        }
      }
      else if (hid > meshId) {
        if (!amIGateway || leaderId != meshId) {
          leaderId = meshId;
          leaderMeshNodeId = myMeshNodeId;
          lastHB = millis();
          lastHB_painless = mesh.getNodeTime();

          setLeaderState(true, "became_leader");

          Serial.printf("Taking leadership over higher leader logical=%u; now leader=%u (mesh node %u)\n",
                        hid, leaderId, leaderMeshNodeId);

          if (leaderId != lastPrintedLeader) {
            Serial.printf("Leader = %u (mesh node %u)\n", leaderId, leaderMeshNodeId);
            lastPrintedLeader = leaderId;
          }

          DynamicJsonDocument doc(64);
          doc["msgType"] = "ctr";
          doc["cmd"] = "heartbeat";
          doc["id"] = meshId;
          doc["sent"] = mesh.getNodeTime();
          String out;
          serializeJson(doc, out);
          mesh.sendBroadcast(out);
        } else {
          lastHB = millis();
        }
      }
      else {
        leaderId = meshId;
        leaderMeshNodeId = myMeshNodeId;
        lastHB = millis();

        if (!amIGateway) {
          Serial.println("Heartbeat matched my logical ID, restoring self as leader");
          setLeaderState(true, "became_leader");
        }

        if (leaderId != lastPrintedLeader) {
          Serial.printf("Leader = %u (mesh node %u)\n", leaderId, leaderMeshNodeId);
          lastPrintedLeader = leaderId;
        }
      }
    }
    else if (strcmp(cmd, "whois") == 0) {
      Serial.printf("Whois from logical=%u mesh=%u\n", id, from);
      if (amIGateway) {
        DynamicJsonDocument doc(64);
        doc["msgType"] = "ctr";
        doc["cmd"] = "heartbeat";
        doc["id"]  = meshId;
        doc["sent"] = mesh.getNodeTime();
        String out;
        serializeJson(doc, out);
        mesh.sendBroadcast(out);
      }
    }
  }
}

void becomeLeaderAndBroadcast(const char* reason) {
  leaderId = meshId;
  leaderMeshNodeId = myMeshNodeId;
  lastHB = millis();
  lastHB_painless = mesh.getNodeTime();

  setLeaderState(true, "became_leader");

  Serial.printf("Taking leadership: reason=%s logical=%u meshNode=%u\n",
                reason, leaderId, leaderMeshNodeId);

  if (leaderId != lastPrintedLeader) {
    Serial.printf("Leader = %u (mesh node %u)\n", leaderId, leaderMeshNodeId);
    lastPrintedLeader = leaderId;
  }

  DynamicJsonDocument doc(64);
  doc["msgType"] = "ctr";
  doc["cmd"] = "heartbeat";
  doc["id"] = meshId;
  doc["sent"] = mesh.getNodeTime();

  String msg;
  serializeJson(doc, msg);
  mesh.sendBroadcast(msg);
}

void sendHeartBeat() {
  if (!amIGateway) return;

  DynamicJsonDocument doc(64);
  doc["msgType"] = "ctr";
  doc["cmd"]     = "heartbeat";
  doc["id"]      = meshId;
  doc["sent"]    = mesh.getNodeTime();
  String msg;
  serializeJson(doc, msg);
  mesh.sendBroadcast(msg);

  lastHB = millis();
  lastHB_painless = mesh.getNodeTime();
}

void electGatewayCheck() {
  if (millis() - bootAt < DISCOVERY_MS) return;

  bool noLeaderSeenRecently = (millis() - lastHB) > HBLOSS;

  if (noLeaderSeenRecently) {
    becomeLeaderAndBroadcast("heartbeat_timeout");
  }
}

void printStatus() {
  // left intentionally empty for now
}

void sendLeaderStatusTask() {
  sendLeaderStatusToPi("periodic_update", false);
}

void onConnChange() {
  std::list<uint32_t> nodes = mesh.getNodeList();

  Serial.printf("Connections changed. Node count=%u\n", (unsigned)nodes.size());

  for (std::list<uint32_t>::iterator it = nodes.begin(); it != nodes.end(); ++it) {
    Serial.printf("  peer mesh node: %u\n", *it);
  }

  if (leaderId != 0) lastHB = millis();

  if (!amIGateway && leaderId == 0) {
    DynamicJsonDocument q(64);
    q["msgType"] = "ctr";
    q["cmd"] = "whois";
    q["id"] = meshId;
    String msg;
    serializeJson(q, msg);
    mesh.sendBroadcast(msg);
  }
}

Task statusTask(1000, TASK_FOREVER, &printStatus);
Task sendHeartBeatThroughMesh(1000, TASK_FOREVER, &sendHeartBeat);
Task electionTask(1000, TASK_FOREVER, &electGatewayCheck);
Task throughputTask(1000, TASK_FOREVER, &printThroughput);

void setup() {
  Serial.begin(BAUD);
  delay(200);

  mesh.setDebugMsgTypes(ERROR);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);

  myMeshNodeId = mesh.getNodeId();
  myId = meshId;

  lastHB = millis();
  bootAt = millis();
  lastHB_painless = mesh.getNodeTime();

  if (leaderId == meshId) {
    leaderMeshNodeId = myMeshNodeId;
  }

  mesh.onReceive(&recieveMessageFromMesh);

  mesh.onNewConnection([](uint32_t nodeId) {
    Serial.printf("Connected: %u\n", nodeId);
    onConnChange();
  });

  mesh.onChangedConnections([]() {
    onConnChange();
  });

  mesh.onNodeDelayReceived([](auto nodeId, auto delay) {
    // optional
  });

  if (leaderId != 0) {
    lastPrintedLeader = leaderId;
    Serial.printf("Leader = %u\n", leaderId);
  }

  Serial.printf("Startup: logical meshId=%u actualMeshNodeId=%u\n", meshId, myMeshNodeId);
  sendLeaderStatusToPi("startup", amIGateway);

  userScheduler.addTask(sendHeartBeatThroughMesh);
  sendHeartBeatThroughMesh.enable();

  userScheduler.addTask(electionTask);
  electionTask.enable();

  userScheduler.addTask(throughputTask);
  throughputTask.enable();

  Task* serialTask = new Task(10, TASK_FOREVER, &readFramedSerialAndHandleData);
  userScheduler.addTask(*serialTask);
  serialTask->enable();

  Task* leaderTask = new Task(8000, TASK_FOREVER, &sendLeaderStatusTask);
  userScheduler.addTask(*leaderTask);
  leaderTask->enable();
}

void loop() {
  mesh.update();
}