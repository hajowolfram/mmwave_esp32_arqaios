#include <stdint.h>

#define MAX_POINTS 100
#define MAX_OBJECTS 100
#define BAUDRATE 921600
#define UART_BUFFER_SIZE 32000

const uint8_t MAGIC_WORD[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

typedef enum
{
    TLV_DETECTED_POINTS = 1,
    TLV_POINT_CLOUD = 1020,
    TLV_OBJ_LIST = 1010,
    TLV_INDEX = 1011,
    TLV_HEIGHT = 1012,
    TLV_PRESENCE = 1021
} tlvType;

typedef struct
{
    uint32_t version;
    uint32_t totalPacketLen;
    uint32_t platform;
    uint32_t frameNumber;
    uint32_t timeCpuCycles;
    uint32_t numDetectedObj;
    uint32_t numTLVs;
    uint32_t subFrameNumber;
} mmwHeader;

typedef struct
{
    float elevationUnit;
    float azimuthUnit;
    float dopplerUnit;
    float rangeUnit;
    float snrUnit;
} pointUnit;

typedef struct
{
    int8_t elevation;
    int8_t azimuth;
    int16_t doppler;
    int16_t range;
    int16_t snr;
} pointObj;

//  __attribute__((__packed__))
typedef struct
{
    uint32_t tid;
    float posX, posY, posZ;
    float velX, velY, velZ;
    float accX, accY, accZ;
    float ec[16];
    float g;
    float confidenceLevel;
} listTlv;

typedef struct
{
    uint8_t targetID;
} indexTlv;

typedef struct
{
    uint32_t present;
} presenceTlv;

typedef struct
{
    uint8_t targetID;
    float maxZ;
    float minZ;
} heightTlv;

typedef struct
{
    mmwHeader header;
    pointObj points[MAX_POINTS];
    pointUnit units;
    int numPoints;
    listTlv objects[MAX_OBJECTS];
    int numObjects;
    indexTlv indices[MAX_POINTS];
    int numIndices;
    heightTlv heights[MAX_OBJECTS];
    int numHeights;
    presenceTlv presence;
    int hasPresence;
} radarFrame;

uint32_t parse_uint32_le(const uint8_t *data);

float parse_float_le(const uint8_t *data);

int16_t parse_int16_le(const uint8_t *data);

void parse_header(const uint8_t *buffer, mmwHeader *header);

void parse_tlv(const uint8_t *buffer, int numTLVs, int offset, int total_len, radarFrame *frame);

int find_magic_word(const uint8_t *buffer, int length);