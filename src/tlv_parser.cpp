#include <string.h>
#include "tlv_parser.h"

uint32_t parse_uint32_le(const uint8_t *data)
{
    return ((uint32_t)data[0]) | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

float parse_float_le(const uint8_t *data)
{
    uint32_t int_val = parse_uint32_le(data);
    return *(float *)&int_val;
}

int16_t parse_int16_le(const uint8_t *data)
{
    return (int16_t)(((uint16_t)data[0]) | ((uint16_t)data[1] << 8));
}

void parse_header(const uint8_t *buffer, mmwHeader *header)
{
    header->version = parse_uint32_le(&buffer[0]);
    header->totalPacketLen = parse_uint32_le(&buffer[4]);
    header->platform = parse_uint32_le(&buffer[8]);
    header->frameNumber = parse_uint32_le(&buffer[12]);
    header->timeCpuCycles = parse_uint32_le(&buffer[16]);
    header->numDetectedObj = parse_uint32_le(&buffer[20]);
    header->numTLVs = parse_uint32_le(&buffer[24]);
    header->subFrameNumber = parse_uint32_le(&buffer[28]);
}

void parse_tlv(const uint8_t *buffer, int numTLVs, int offset, int total_len, radarFrame *frame)
{
    for (int i = 0; i < numTLVs; i++)
    {
        if (offset + 8 > total_len)
        {
            // Serial.println("[ERROR] TLV header extends beyond buffer\n");
            return;
        }

        uint32_t type = parse_uint32_le(&buffer[offset]);
        uint32_t length = parse_uint32_le(&buffer[offset + 4]);
        offset += 8;

        if (offset + length > total_len)
        {
            // Serial.println("[ERROR] TLV payload extends beyond buffer");
            return;
        }

        const uint8_t *payload = &buffer[offset];

        switch (type)
        {
        case TLV_POINT_CLOUD:
        {
            if (length < 20)
            {
                // Serial.println("[ERROR] Point cloud TLV too short");
                break;
            }

            frame->units.elevationUnit = parse_float_le(&payload[0]);
            frame->units.azimuthUnit = parse_float_le(&payload[4]);
            frame->units.dopplerUnit = parse_float_le(&payload[8]);
            frame->units.rangeUnit = parse_float_le(&payload[12]);
            frame->units.snrUnit = parse_float_le(&payload[16]);

            const uint8_t *point_data = payload + 20;
            int point_payload_len = length - 20;
            int numPoints = point_payload_len / 8;

            if (numPoints > MAX_POINTS)
            {
                // Serial.println("[WARNING] Truncating points \n");
                numPoints = MAX_POINTS;
            }

            frame->numPoints = numPoints;

            for (int p = 0; p < numPoints; p++)
            {
                const uint8_t *pt_data = point_data + (p * 8);
                frame->points[p].elevation = (int8_t)pt_data[0];
                frame->points[p].azimuth = (int8_t)pt_data[1];
                frame->points[p].doppler = parse_int16_le(&pt_data[2]);
                frame->points[p].range = parse_int16_le(&pt_data[4]);
                frame->points[p].snr = parse_int16_le(&pt_data[6]);
            }
            break;
        }
        case TLV_DETECTED_POINTS:
        {
            frame->numPoints = length / sizeof(pointObj);
            if (frame->numPoints > MAX_POINTS)
                frame->numPoints = MAX_POINTS;
            memcpy(frame->points, payload, frame->numPoints * sizeof(pointObj));
            break;
        }
        case TLV_OBJ_LIST:
        {
            frame->numObjects = length / sizeof(listTlv);
            if (frame->numObjects > MAX_OBJECTS)
                frame->numObjects = MAX_OBJECTS;
            memcpy(frame->objects, payload, frame->numObjects * sizeof(listTlv));
            break;
        }
        case TLV_INDEX:
        {
            frame->numIndices = length / sizeof(indexTlv);
            if (frame->numIndices > MAX_POINTS)
                frame->numIndices = MAX_POINTS;
            memcpy(frame->indices, payload, frame->numIndices * sizeof(indexTlv));
            break;
        }
        case TLV_HEIGHT:
        {
            frame->numHeights = length / sizeof(heightTlv);
            if (frame->numHeights > MAX_OBJECTS)
                frame->numHeights = MAX_OBJECTS;
            memcpy(frame->heights, payload, frame->numHeights * sizeof(heightTlv));
            break;
        }
        case TLV_PRESENCE:
        {
            if (length >= sizeof(presenceTlv))
            {
                frame->presence.present = parse_uint32_le(payload);
                frame->hasPresence = 1;
            }
            break;
        }
        default:
            // Serial.println("[WARNING] Unknown TLV type");
            break;
        }
        offset += length;
    }
}

int find_magic_word(const uint8_t *buffer, int length)
{
    for (int i = 0; i <= length - 8; i++)
    {
        if (memcmp(&buffer[i], MAGIC_WORD, 8) == 0)
            return i;
    }
    return -1;
}