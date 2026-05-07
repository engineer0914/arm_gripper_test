# 교수님 작성해주신 원본 코드
# 처음에 열린 상태로 놓고 대기하다가,
# 1이면 집고, 중간에 안잡힌것 같으면 다시 놓기
# 0이면 열기



#include <Arduino.h>

#define DEBUG_PORT   Serial
#define DXL_PORT     Serial3

// ============================================================
// User Settings
// ============================================================
#define DXL_ID                 10
#define DXL_BAUD               57600

// [Edit] 목표 위치(예시). 하드스톱에 걸면 떨림이 커지므로 끝단에서 약간 빼서 시작
#define OPEN_POS               30
#define CLOSE_POS              710

// [Edit] 전류(단위: RH-P12 control table unit)
#define MOVE_CURRENT_CLOSE     220     // 닫을 때 힘(전류) 제한/목표
#define HOLD_CURRENT_CLOSE     70      // 파지 성공 후 유지 전류(떨림/발열 저감)
#define MOVE_CURRENT_OPEN      120     // 열 때 이동 전류

// [Edit] 파지 판정 전류 임계값 + 유지시간
#define GRASP_CURRENT_THRESHOLD  160    // |PresentCurrent| >= 이 값이면 "접촉/파지"로 판단
#define GRASP_HOLD_MS            300    // 위 조건이 이 시간(연속) 유지되면 파지 성공

// [Edit] 타임아웃
#define GRASP_MAX_MS           3500    // 닫기 + 파지 판정 최대 시간
#define OPEN_MAX_MS            3000

// [Edit] 위치 도달 판정(OPEN 동작에서만 사용)
#define POS_EPS                10

// [Edit] 통신 타임아웃
#define RX_TIMEOUT_MS          30

// ============================================================
// OpenCR DXL Power
// ============================================================
#define DXL_POWER_ENABLE()     digitalWrite(BDPIN_DXL_PWR_EN, HIGH)

// ============================================================
// Dynamixel Protocol 2.0
// ============================================================
#define INST_PING              0x01
#define INST_READ              0x02
#define INST_WRITE             0x03

// ============================================================
// RH-P12-RN(A) Control Table (e-Manual 기반)
// ============================================================
#define ADDR_TORQUE_ENABLE     512     // 1 byte
#define ADDR_BUS_WATCHDOG      546     // 1 byte
#define ADDR_GOAL_CURRENT      550     // 2 bytes
#define ADDR_GOAL_POSITION     564     // 4 bytes
#define ADDR_PRESENT_CURRENT   574     // 2 bytes
#define ADDR_PRESENT_POSITION  580     // 4 bytes

// ============================================================
// CRC (Protocol 2.0)
// ============================================================
uint16_t updateCRC(uint16_t crc_accum, const uint8_t *data_blk_ptr, uint16_t data_blk_size)
{
  uint16_t i, j;
  static const uint16_t crc_table[256] =
  {
    0x0000,0x8005,0x800F,0x000A,0x801B,0x001E,0x0014,0x8011,
    0x8033,0x0036,0x003C,0x8039,0x0028,0x802D,0x8027,0x0022,
    0x8063,0x0066,0x006C,0x8069,0x0078,0x807D,0x8077,0x0072,
    0x0050,0x8055,0x805F,0x005A,0x804B,0x004E,0x0044,0x8041,
    0x80C3,0x00C6,0x00CC,0x80C9,0x00D8,0x80DD,0x80D7,0x00D2,
    0x00F0,0x80F5,0x80FF,0x00FA,0x80EB,0x00EE,0x00E4,0x80E1,
    0x00A0,0x80A5,0x80AF,0x00AA,0x80BB,0x00BE,0x00B4,0x80B1,
    0x8093,0x0096,0x009C,0x8099,0x0088,0x808D,0x8087,0x0082,
    0x8183,0x0186,0x018C,0x8189,0x0198,0x819D,0x8197,0x0192,
    0x01B0,0x81B5,0x81BF,0x01BA,0x81AB,0x01AE,0x01A4,0x81A1,
    0x01E0,0x81E5,0x81EF,0x01EA,0x81FB,0x01FE,0x01F4,0x81F1,
    0x81D3,0x01D6,0x01DC,0x81D9,0x01C8,0x81CD,0x81C7,0x01C2,
    0x0140,0x8145,0x814F,0x014A,0x815B,0x015E,0x0154,0x8151,
    0x8173,0x0176,0x017C,0x8179,0x0168,0x816D,0x8167,0x0162,
    0x8123,0x0126,0x012C,0x8129,0x0138,0x813D,0x8137,0x0132,
    0x0110,0x8115,0x811F,0x011A,0x810B,0x010E,0x0104,0x8101,
    0x8303,0x0306,0x030C,0x8309,0x0318,0x831D,0x8317,0x0312,
    0x0330,0x8335,0x833F,0x033A,0x832B,0x032E,0x0324,0x8321,
    0x0360,0x8365,0x836F,0x036A,0x837B,0x037E,0x0374,0x8371,
    0x8353,0x0356,0x035C,0x8359,0x0348,0x834D,0x8347,0x0342,
    0x03C0,0x83C5,0x83CF,0x03CA,0x83DB,0x03DE,0x03D4,0x83D1,
    0x83F3,0x03F6,0x03FC,0x83F9,0x03E8,0x83ED,0x83E7,0x03E2,
    0x83A3,0x03A6,0x03AC,0x83A9,0x03B8,0x83BD,0x83B7,0x03B2,
    0x0390,0x8395,0x839F,0x039A,0x838B,0x038E,0x0384,0x8381,
    0x0280,0x8285,0x828F,0x028A,0x829B,0x029E,0x0294,0x8291,
    0x82B3,0x02B6,0x02BC,0x82B9,0x02A8,0x82AD,0x82A7,0x02A2,
    0x82E3,0x02E6,0x02EC,0x82E9,0x02F8,0x82FD,0x82F7,0x02F2,
    0x02D0,0x82D5,0x82DF,0x02DA,0x82CB,0x02CE,0x02C4,0x82C1,
    0x8243,0x0246,0x024C,0x8249,0x0258,0x825D,0x8257,0x0252,
    0x0270,0x8275,0x827F,0x027A,0x826B,0x026E,0x0264,0x8261,
    0x0220,0x8225,0x822F,0x022A,0x823B,0x023E,0x0234,0x8231,
    0x8213,0x0216,0x021C,0x8219,0x0208,0x820D,0x8207,0x0202
  };

  for (j = 0; j < data_blk_size; j++)
  {
    i = ((uint16_t)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xFF;
    crc_accum = (crc_accum << 8) ^ crc_table[i];
  }

  return crc_accum;
}

// ============================================================
// Half-duplex TX/RX helpers (usb_to_dxl 방식)
// ============================================================
void dxlFlushRx()
{
  while (DXL_PORT.available())
  {
    (void)DXL_PORT.read();
  }
}

int dxlReceive(uint8_t *rx, int rx_max, uint32_t timeout_ms)
{
  int rx_len = 0;
  uint32_t t0 = millis();

  while ((millis() - t0) < timeout_ms)
  {
    while (DXL_PORT.available() && rx_len < rx_max)
    {
      rx[rx_len++] = (uint8_t)DXL_PORT.read();
    }

    if (rx_len >= 12)
    {
      uint16_t len = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8);
      int total = 7 + (int)len;
      if (rx_len >= total)
      {
        return rx_len;
      }
    }
  }

  return rx_len;
}

bool dxlSendPacket(const uint8_t *tx, int tx_len)
{
  dxlFlushRx();

  drv_dxl_tx_enable(TRUE);
  DXL_PORT.write(tx, tx_len);
  DXL_PORT.flush();
  drv_dxl_tx_enable(FALSE);

  return true;
}

bool dxlBuildAndSend(uint8_t id, uint8_t inst, const uint8_t *params, uint16_t param_len)
{
  uint16_t len = (uint16_t)(1 + param_len + 2);

  uint8_t tx[128];
  int idx = 0;

  tx[idx++] = 0xFF;
  tx[idx++] = 0xFF;
  tx[idx++] = 0xFD;
  tx[idx++] = 0x00;
  tx[idx++] = id;
  tx[idx++] = (uint8_t)(len & 0xFF);
  tx[idx++] = (uint8_t)((len >> 8) & 0xFF);
  tx[idx++] = inst;

  for (uint16_t i = 0; i < param_len; i++)
  {
    tx[idx++] = params[i];
  }

  uint16_t crc = updateCRC(0, tx, (uint16_t)idx);
  tx[idx++] = (uint8_t)(crc & 0xFF);
  tx[idx++] = (uint8_t)((crc >> 8) & 0xFF);

  return dxlSendPacket(tx, idx);
}

// ============================================================
// Basic DXL R/W
// ============================================================
bool dxlPing(uint8_t id)
{
  if (!dxlBuildAndSend(id, INST_PING, nullptr, 0))
  {
    return false;
  }

  uint8_t rx[64];
  int rx_len = dxlReceive(rx, (int)sizeof(rx), RX_TIMEOUT_MS);

  if (rx_len < 12) { return false; }
  if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != 0xFD || rx[3] != 0x00) { return false; }

  return (rx[4] == id);
}

bool dxlWrite1(uint8_t id, uint16_t addr, uint8_t data)
{
  uint8_t p[3];
  p[0] = (uint8_t)(addr & 0xFF);
  p[1] = (uint8_t)((addr >> 8) & 0xFF);
  p[2] = data;

  if (!dxlBuildAndSend(id, INST_WRITE, p, (uint16_t)sizeof(p))) { return false; }

  uint8_t rx[64];
  return (dxlReceive(rx, (int)sizeof(rx), RX_TIMEOUT_MS) >= 12);
}

bool dxlWrite2(uint8_t id, uint16_t addr, uint16_t data)
{
  uint8_t p[4];
  p[0] = (uint8_t)(addr & 0xFF);
  p[1] = (uint8_t)((addr >> 8) & 0xFF);
  p[2] = (uint8_t)(data & 0xFF);
  p[3] = (uint8_t)((data >> 8) & 0xFF);

  if (!dxlBuildAndSend(id, INST_WRITE, p, (uint16_t)sizeof(p))) { return false; }

  uint8_t rx[64];
  return (dxlReceive(rx, (int)sizeof(rx), RX_TIMEOUT_MS) >= 12);
}

bool dxlWrite4(uint8_t id, uint16_t addr, uint32_t data)
{
  uint8_t p[6];
  p[0] = (uint8_t)(addr & 0xFF);
  p[1] = (uint8_t)((addr >> 8) & 0xFF);
  p[2] = (uint8_t)(data & 0xFF);
  p[3] = (uint8_t)((data >> 8) & 0xFF);
  p[4] = (uint8_t)((data >> 16) & 0xFF);
  p[5] = (uint8_t)((data >> 24) & 0xFF);

  if (!dxlBuildAndSend(id, INST_WRITE, p, (uint16_t)sizeof(p))) { return false; }

  uint8_t rx[64];
  return (dxlReceive(rx, (int)sizeof(rx), RX_TIMEOUT_MS) >= 12);
}

bool dxlRead(uint8_t id, uint16_t addr, uint16_t size, uint8_t *out, uint16_t out_max)
{
  uint8_t p[4];
  p[0] = (uint8_t)(addr & 0xFF);
  p[1] = (uint8_t)((addr >> 8) & 0xFF);
  p[2] = (uint8_t)(size & 0xFF);
  p[3] = (uint8_t)((size >> 8) & 0xFF);

  if (!dxlBuildAndSend(id, INST_READ, p, (uint16_t)sizeof(p))) { return false; }

  uint8_t rx[128];
  int rx_len = dxlReceive(rx, (int)sizeof(rx), RX_TIMEOUT_MS);

  if (rx_len < 12) { return false; }
  if (rx[0] != 0xFF || rx[1] != 0xFF || rx[2] != 0xFD || rx[3] != 0x00) { return false; }
  if (rx[4] != id) { return false; }

  uint16_t len = (uint16_t)rx[5] | ((uint16_t)rx[6] << 8);
  uint16_t params_len = (uint16_t)(len - 1 - 1 - 2);

  if (params_len < size) { return false; }
  if (size > out_max) { return false; }

  for (uint16_t i = 0; i < size; i++)
  {
    out[i] = rx[9 + i];
  }

  return true;
}

bool dxlRead2(uint8_t id, uint16_t addr, int16_t *value)
{
  uint8_t buf[2];
  if (!dxlRead(id, addr, 2, buf, 2)) { return false; }
  *value = (int16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
  return true;
}

bool dxlRead4(uint8_t id, uint16_t addr, int32_t *value)
{
  uint8_t buf[4];
  if (!dxlRead(id, addr, 4, buf, 4)) { return false; }
  *value = (int32_t)((uint32_t)buf[0]
                    | ((uint32_t)buf[1] << 8)
                    | ((uint32_t)buf[2] << 16)
                    | ((uint32_t)buf[3] << 24));
  return true;
}

// ============================================================
// RH-P12 Helpers
// ============================================================
void rhp12ClearWatchdog()
{
  (void)dxlWrite1(DXL_ID, ADDR_BUS_WATCHDOG, 0);
}

void rhp12TorqueOn()
{
  (void)dxlWrite1(DXL_ID, ADDR_TORQUE_ENABLE, 1);
}

void rhp12TorqueOff()
{
  (void)dxlWrite1(DXL_ID, ADDR_TORQUE_ENABLE, 0);
}

// [Edit] OPEN 동작: 목표 도달 후 떨림 완화 + Torque OFF
void rhp12OpenAndTorqueOff()
{
  DEBUG_PORT.println("[CMD] OPEN -> reach then TORQUE OFF");

  rhp12ClearWatchdog();
  delay(5);

  rhp12TorqueOn();
  delay(5);

  (void)dxlWrite2(DXL_ID, ADDR_GOAL_CURRENT, (uint16_t)MOVE_CURRENT_OPEN);
  (void)dxlWrite4(DXL_ID, ADDR_GOAL_POSITION, (uint32_t)OPEN_POS);

  uint32_t t0 = millis();
  while ((millis() - t0) < OPEN_MAX_MS)
  {
    int32_t pos = 0;
    if (dxlRead4(DXL_ID, ADDR_PRESENT_POSITION, &pos))
    {
      int32_t err = pos - (int32_t)OPEN_POS;
      if (err < 0) { err = -err; }

      if (err <= POS_EPS)
      {
        // [Edit] 도달 시 Goal=Present로 재설정(잔떨림 감소)
        (void)dxlWrite4(DXL_ID, ADDR_GOAL_POSITION, (uint32_t)pos);
        delay(50);

        rhp12TorqueOff();
        DEBUG_PORT.println("[OPEN] Done. Torque OFF.");
        return;
      }
    }
    delay(10);
  }

  // [Edit] 타임아웃이어도 토크 OFF 정책
  rhp12TorqueOff();
  DEBUG_PORT.println("[OPEN] Timeout. Torque OFF anyway.");
}

// [Edit] CLOSE + 전류 기반 파지:
// - CLOSE_POS까지 닫기 시도
// - |PresentCurrent| >= THRESHOLD가 HOLD_MS 동안 연속 유지되면 파지 성공
// - 성공: present_pos를 goal로 고정 + hold current로 낮춤 + torque ON 유지
// - 실패: OPEN + torque OFF
void rhp12GraspByCurrent()
{
  DEBUG_PORT.println("[CMD] GRASP (current condition)");

  rhp12ClearWatchdog();
  delay(5);

  rhp12TorqueOn();
  delay(5);

  // 1) 닫기 시작
  (void)dxlWrite2(DXL_ID, ADDR_GOAL_CURRENT, (uint16_t)MOVE_CURRENT_CLOSE);
  (void)dxlWrite4(DXL_ID, ADDR_GOAL_POSITION, (uint32_t)CLOSE_POS);

  uint32_t t0 = millis();
  uint32_t grasp_start_ms = 0;    // 조건 성립 시작 시각(0이면 아직 미성립)
  int32_t hold_pos = 0;

  while ((millis() - t0) < GRASP_MAX_MS)
  {
    int16_t cur = 0;
    int32_t pos = 0;

    bool ok_c = dxlRead2(DXL_ID, ADDR_PRESENT_CURRENT, &cur);
    bool ok_p = dxlRead4(DXL_ID, ADDR_PRESENT_POSITION, &pos);

    if (ok_c && ok_p)
    {
      int16_t abs_c = (cur >= 0) ? cur : (int16_t)(-cur);

      // DEBUG 로그(원치 않으면 주석 처리)
      DEBUG_PORT.print("  I=");
      DEBUG_PORT.print(cur);
      DEBUG_PORT.print("  Pos=");
      DEBUG_PORT.println(pos);

      // 2) 파지 조건 체크
      if (abs_c >= GRASP_CURRENT_THRESHOLD)
      {
        if (grasp_start_ms == 0)
        {
          // [Edit] 조건 처음 성립 시각 기록
          grasp_start_ms = millis();
          hold_pos = pos;
        }
        else
        {
          // [Edit] 조건이 연속으로 유지되는지 시간 확인
          if ((millis() - grasp_start_ms) >= GRASP_HOLD_MS)
          {
            // 3) 파지 성공: hold
            DEBUG_PORT.println("[GRASP] Condition satisfied. HOLD.");

            // [Edit] 홀드 전류로 낮추고 현재 위치(또는 hold_pos)로 goal 고정
            (void)dxlWrite2(DXL_ID, ADDR_GOAL_CURRENT, (uint16_t)HOLD_CURRENT_CLOSE);
            (void)dxlWrite4(DXL_ID, ADDR_GOAL_POSITION, (uint32_t)hold_pos);

            // [Edit] 떨림 방지: 한 번 더 현재 위치로 goal 갱신
            int32_t pos_now = 0;
            if (dxlRead4(DXL_ID, ADDR_PRESENT_POSITION, &pos_now))
            {
              (void)dxlWrite4(DXL_ID, ADDR_GOAL_POSITION, (uint32_t)pos_now);
            }

            DEBUG_PORT.println("[GRASP] Torque remains ON.");
            return;
          }
        }
      }
      else
      {
        // [Edit] 조건이 깨지면 연속 유지 판정 리셋
        grasp_start_ms = 0;
      }
    }

    delay(10);
  }

  // 4) 실패: OPEN + torque OFF
  DEBUG_PORT.println("[GRASP] Failed (timeout or no condition). Releasing...");
  rhp12OpenAndTorqueOff();
}

// ============================================================
// Setup / Loop
// ============================================================
void setup()
{
  DEBUG_PORT.begin(115200);
  while (!DEBUG_PORT) { ; }

  DXL_PORT.begin(DXL_BAUD);

  pinMode(BDPIN_DXL_PWR_EN, OUTPUT);
  DXL_POWER_ENABLE();
  delay(100);

  drv_dxl_tx_enable(FALSE);
  delay(10);

  DEBUG_PORT.println("=== RH-P12-RN(A) Current-based Grasp ===");
  DEBUG_PORT.println("  '1' : CLOSE to CLOSE_POS and verify grasp-current for HOLD_MS");
  DEBUG_PORT.println("        If fail -> OPEN and Torque OFF");
  DEBUG_PORT.println("  '0' : OPEN and Torque OFF (same as before)");

  if (!dxlPing(DXL_ID))
  {
    DEBUG_PORT.println("[Init] PING FAIL");
    return;
  }

  DEBUG_PORT.println("[Init] PING OK");

  // [Edit] 초기 상태: OPEN + torque OFF
  rhp12OpenAndTorqueOff();
}

void loop()
{
  if (DEBUG_PORT.available())
  {
    char c = (char)DEBUG_PORT.read();

    if (c == '1')
    {
      rhp12GraspByCurrent();
    }
    else if (c == '0')
    {
      rhp12OpenAndTorqueOff();
    }
  }

  delay(10);
}
