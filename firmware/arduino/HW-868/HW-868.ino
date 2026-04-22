#include <SPI.h>

#define uchar unsigned char
#define uint unsigned int

#define MAX_LEN 16

// ===================== 引脚定义 =====================
const int chipSelectPin = 53;   // RC522 SDA/SS -> Mega D53
const int NRSTPD = 5;           // RC522 RST -> Mega D5（按你的实际接线修改）
const int ledPin = 41;          // 外接LED -> Mega D41

// ===================== MFRC522 命令定义 =====================
#define PCD_IDLE 0x00
#define PCD_AUTHENT 0x0E
#define PCD_RECEIVE 0x08
#define PCD_TRANSMIT 0x04
#define PCD_TRANSCEIVE 0x0C
#define PCD_RESETPHASE 0x0F
#define PCD_CALCCRC 0x03

#define PICC_REQIDL 0x26
#define PICC_REQALL 0x52
#define PICC_ANTICOLL 0x93
#define PICC_SElECTTAG 0x93
#define PICC_AUTHENT1A 0x60
#define PICC_AUTHENT1B 0x61
#define PICC_READ 0x30
#define PICC_WRITE 0xA0
#define PICC_DECREMENT 0xC0
#define PICC_INCREMENT 0xC1
#define PICC_RESTORE 0xC2
#define PICC_TRANSFER 0xB0
#define PICC_HALT 0x50

#define MI_OK 0
#define MI_NOTAGERR 1
#define MI_ERR 2

// ===================== MFRC522 寄存器定义 =====================
// Page 0: Command and Status
#define Reserved00 0x00
#define CommandReg 0x01
#define CommIEnReg 0x02
#define DivlEnReg 0x03
#define CommIrqReg 0x04
#define DivIrqReg 0x05
#define ErrorReg 0x06
#define Status1Reg 0x07
#define Status2Reg 0x08
#define FIFODataReg 0x09
#define FIFOLevelReg 0x0A
#define WaterLevelReg 0x0B
#define ControlReg 0x0C
#define BitFramingReg 0x0D
#define CollReg 0x0E
#define Reserved01 0x0F

// Page 1: Command
#define Reserved10 0x10
#define ModeReg 0x11
#define TxModeReg 0x12
#define RxModeReg 0x13
#define TxControlReg 0x14
#define TxAutoReg 0x15
#define TxSelReg 0x16
#define RxSelReg 0x17
#define RxThresholdReg 0x18
#define DemodReg 0x19
#define Reserved11 0x1A
#define Reserved12 0x1B
#define MifareReg 0x1C
#define Reserved13 0x1D
#define Reserved14 0x1E
#define SerialSpeedReg 0x1F

// Page 2: CFG
#define Reserved20 0x20
#define CRCResultRegM 0x21
#define CRCResultRegL 0x22
#define Reserved21 0x23
#define ModWidthReg 0x24
#define Reserved22 0x25
#define RFCfgReg 0x26
#define GsNReg 0x27
#define CWGsPReg 0x28
#define ModGsPReg 0x29
#define TModeReg 0x2A
#define TPrescalerReg 0x2B
#define TReloadRegH 0x2C
#define TReloadRegL 0x2D
#define TCounterValueRegH 0x2E
#define TCounterValueRegL 0x2F

// Page 3: TestRegister
#define Reserved30 0x30
#define TestSel1Reg 0x31
#define TestSel2Reg 0x32
#define TestPinEnReg 0x33
#define TestPinValueReg 0x34
#define TestBusReg 0x35
#define AutoTestReg 0x36
#define VersionReg 0x37
#define AnalogTestReg 0x38
#define TestDAC1Reg 0x39
#define TestDAC2Reg 0x3A
#define TestADCReg 0x3B
#define Reserved31 0x3C
#define Reserved32 0x3D
#define Reserved33 0x3E
#define Reserved34 0x3F

uchar serNum[5];

// ===================== 函数声明 =====================
void ShowCardID(uchar *id);
void ShowCardType(uchar *type);
void Write_MFRC522(uchar addr, uchar val);
uchar Read_MFRC522(uchar addr);
void SetBitMask(uchar reg, uchar mask);
void ClearBitMask(uchar reg, uchar mask);
void AntennaOn(void);
void AntennaOff(void);
void MFRC522_Reset(void);
void MFRC522_Init(void);
uchar MFRC522_Request(uchar reqMode, uchar *TagType);
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen);
uchar MFRC522_Anticoll(uchar *serNum);
void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData);
uchar MFRC522_Write(uchar blockAddr, uchar *writeData);
void MFRC522_Halt(void);

// ===================== setup =====================
void setup()
{
    Serial.begin(115200);
    SPI.begin();

    digitalWrite(47, HIGH);  // 改成 LOW

    pinMode(chipSelectPin, OUTPUT);
    digitalWrite(chipSelectPin, LOW);

    pinMode(NRSTPD, OUTPUT);
    digitalWrite(NRSTPD, HIGH);

    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);

    Serial.print("Start program");

    MFRC522_Init();
}

// ===================== loop =====================
void loop()
{
    uchar status;
    uchar str[MAX_LEN];

    // 默认无卡时熄灯
    digitalWrite(ledPin, LOW);

    // 搜索卡
    status = MFRC522_Request(PICC_REQIDL, str);
    if (status != MI_OK)
    {
        delay(50);
        return;
    }

    // 读到任意卡，点亮LED
    digitalWrite(ledPin, HIGH);

    // 输出卡类型
    ShowCardType(str);

    // 防冲突，读取UID
    status = MFRC522_Anticoll(str);
    if (status == MI_OK)
    {
        memcpy(serNum, str, 5);
        Serial.print("Card UID: ");
        ShowCardID(serNum);
    }
    else
    {
        Serial.println("Read UID failed.");
    }

    MFRC522_Halt();
    delay(300);
}

/*
 * 显示卡号 UID
 */
void ShowCardID(uchar *id)
{
    int IDlen = 4;
    for (int i = 0; i < IDlen; i++)
    {
        Serial.print(0x0F & (id[i] >> 4), HEX);
        Serial.print(0x0F & id[i], HEX);
    }
    Serial.println("");
}

/*
 * 显示卡类型
 */
void ShowCardType(uchar *type)
{
    Serial.print("Card type: ");
    if (type[0] == 0x04 && type[1] == 0x00)
        Serial.println("MFOne-S50");
    else if (type[0] == 0x02 && type[1] == 0x00)
        Serial.println("MFOne-S70");
    else if (type[0] == 0x44 && type[1] == 0x00)
        Serial.println("MF-UltraLight");
    else if (type[0] == 0x08 && type[1] == 0x00)
        Serial.println("MF-Pro");
    else if (type[0] == 0x44 && type[1] == 0x03)
        Serial.println("MF Desire");
    else
        Serial.println("Unknown");
}

/*
 * 写寄存器
 */
void Write_MFRC522(uchar addr, uchar val)
{
    digitalWrite(chipSelectPin, LOW);
    SPI.transfer((addr << 1) & 0x7E);
    SPI.transfer(val);
    digitalWrite(chipSelectPin, HIGH);
}

/*
 * 读寄存器
 */
uchar Read_MFRC522(uchar addr)
{
    uchar val;
    digitalWrite(chipSelectPin, LOW);
    SPI.transfer(((addr << 1) & 0x7E) | 0x80);
    val = SPI.transfer(0x00);
    digitalWrite(chipSelectPin, HIGH);
    return val;
}

/*
 * 置位寄存器位
 */
void SetBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp | mask);
}

/*
 * 清零寄存器位
 */
void ClearBitMask(uchar reg, uchar mask)
{
    uchar tmp;
    tmp = Read_MFRC522(reg);
    Write_MFRC522(reg, tmp & (~mask));
}

/*
 * 打开天线
 */
void AntennaOn(void)
{
    uchar temp;
    temp = Read_MFRC522(TxControlReg);
    if (!(temp & 0x03))
    {
        SetBitMask(TxControlReg, 0x03);
    }
}

/*
 * 关闭天线
 */
void AntennaOff(void)
{
    ClearBitMask(TxControlReg, 0x03);
}

/*
 * 复位 RC522
 */
void MFRC522_Reset(void)
{
    Write_MFRC522(CommandReg, PCD_RESETPHASE);
}

/*
 * 初始化 RC522
 */
void MFRC522_Init(void)
{
    digitalWrite(NRSTPD, HIGH);

    MFRC522_Reset();

    Write_MFRC522(TModeReg, 0x8D);
    Write_MFRC522(TPrescalerReg, 0x3E);
    Write_MFRC522(TReloadRegL, 30);
    Write_MFRC522(TReloadRegH, 0);

    Write_MFRC522(TxAutoReg, 0x40);
    Write_MFRC522(ModeReg, 0x3D);

    AntennaOn();
}

/*
 * 请求卡
 */
uchar MFRC522_Request(uchar reqMode, uchar *TagType)
{
    uchar status;
    uint backBits;

    Write_MFRC522(BitFramingReg, 0x07);

    TagType[0] = reqMode;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, TagType, 1, TagType, &backBits);

    if ((status != MI_OK) || (backBits != 0x10))
    {
        status = MI_ERR;
    }

    return status;
}

/*
 * RC522 和卡通信
 */
uchar MFRC522_ToCard(uchar command, uchar *sendData, uchar sendLen, uchar *backData, uint *backLen)
{
    uchar status = MI_ERR;
    uchar irqEn = 0x00;
    uchar waitIRq = 0x00;
    uchar lastBits;
    uchar n;
    uint i;

    switch (command)
    {
    case PCD_AUTHENT:
        irqEn = 0x12;
        waitIRq = 0x10;
        break;

    case PCD_TRANSCEIVE:
        irqEn = 0x77;
        waitIRq = 0x30;
        break;

    default:
        break;
    }

    Write_MFRC522(CommIEnReg, irqEn | 0x80);
    ClearBitMask(CommIrqReg, 0x80);
    SetBitMask(FIFOLevelReg, 0x80);

    Write_MFRC522(CommandReg, PCD_IDLE);

    for (i = 0; i < sendLen; i++)
    {
        Write_MFRC522(FIFODataReg, sendData[i]);
    }

    Write_MFRC522(CommandReg, command);
    if (command == PCD_TRANSCEIVE)
    {
        SetBitMask(BitFramingReg, 0x80);
    }

    i = 2000;
    do
    {
        n = Read_MFRC522(CommIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x01) && !(n & waitIRq));

    ClearBitMask(BitFramingReg, 0x80);

    if (i != 0)
    {
        if (!(Read_MFRC522(ErrorReg) & 0x1B))
        {
            status = MI_OK;
            if (n & irqEn & 0x01)
            {
                status = MI_NOTAGERR;
            }

            if (command == PCD_TRANSCEIVE)
            {
                n = Read_MFRC522(FIFOLevelReg);
                lastBits = Read_MFRC522(ControlReg) & 0x07;
                if (lastBits)
                {
                    *backLen = (n - 1) * 8 + lastBits;
                }
                else
                {
                    *backLen = n * 8;
                }

                if (n == 0)
                {
                    n = 1;
                }
                if (n > MAX_LEN)
                {
                    n = MAX_LEN;
                }

                for (i = 0; i < n; i++)
                {
                    backData[i] = Read_MFRC522(FIFODataReg);
                }
            }
        }
        else
        {
            status = MI_ERR;
        }
    }

    return status;
}

/*
 * 防冲突并读取卡UID
 */
uchar MFRC522_Anticoll(uchar *serNum)
{
    uchar status;
    uchar i;
    uchar serNumCheck = 0;
    uint unLen;

    Write_MFRC522(BitFramingReg, 0x00);

    serNum[0] = PICC_ANTICOLL;
    serNum[1] = 0x20;
    status = MFRC522_ToCard(PCD_TRANSCEIVE, serNum, 2, serNum, &unLen);

    if (status == MI_OK)
    {
        for (i = 0; i < 4; i++)
        {
            serNumCheck ^= serNum[i];
        }
        if (serNumCheck != serNum[i])
        {
            status = MI_ERR;
        }
    }

    return status;
}

/*
 * CRC 计算
 */
void CalulateCRC(uchar *pIndata, uchar len, uchar *pOutData)
{
    uchar i, n;

    ClearBitMask(DivIrqReg, 0x04);
    SetBitMask(FIFOLevelReg, 0x80);

    for (i = 0; i < len; i++)
    {
        Write_MFRC522(FIFODataReg, *(pIndata + i));
    }

    Write_MFRC522(CommandReg, PCD_CALCCRC);

    i = 0xFF;
    do
    {
        n = Read_MFRC522(DivIrqReg);
        i--;
    } while ((i != 0) && !(n & 0x04));

    pOutData[0] = Read_MFRC522(CRCResultRegL);
    pOutData[1] = Read_MFRC522(CRCResultRegM);
}

/*
 * 写块数据
 */
uchar MFRC522_Write(uchar blockAddr, uchar *writeData)
{
    uchar status;
    uint recvBits;
    uchar i;
    uchar buff[18];

    buff[0] = PICC_WRITE;
    buff[1] = blockAddr;
    CalulateCRC(buff, 2, &buff[2]);
    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &recvBits);

    if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
    {
        status = MI_ERR;
    }

    if (status == MI_OK)
    {
        for (i = 0; i < 16; i++)
        {
            buff[i] = *(writeData + i);
        }
        CalulateCRC(buff, 16, &buff[16]);
        status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 18, buff, &recvBits);

        if ((status != MI_OK) || (recvBits != 4) || ((buff[0] & 0x0F) != 0x0A))
        {
            status = MI_ERR;
        }
    }

    return status;
}

/*
 * 让卡进入休眠
 */
void MFRC522_Halt(void)
{
    uchar status;
    uint unLen;
    uchar buff[4];

    buff[0] = PICC_HALT;
    buff[1] = 0;
    CalulateCRC(buff, 2, &buff[2]);

    status = MFRC522_ToCard(PCD_TRANSCEIVE, buff, 4, buff, &unLen);
}