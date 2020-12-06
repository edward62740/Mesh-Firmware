#include "mesh.h"
#include "utils.h"
#include "sdk_common.h"
#include "sdk_config.h"

#include "boards.h"

#include "nrf_esb.h"
#include "nrf_esb_error_codes.h"

#include "nrf_drv_timer.h"

#include "nrf_delay.h"
#include <stdio.h>

static app_mesh_cmd_handler_t m_app_cmd_handler;
uint8_t cmd_parse_response(char* text,uint8_t*data,uint8_t size)
{
    uint8_t length;
    switch(data[0])
    {
        case MESH_cmd_node_id_set:
        {
            //TODO persistance of parameters
            length = sprintf(text,"cmd:set_node_id;set:%u;get:%u",data[1],data[2]);
        }
        break;
        case MESH_cmd_node_id_get:
        {
            length = sprintf(text,"cmd:get_node_id;node_id:%u",data[1]);
        }
        break;
        case MESH_cmd_rf_chan_set:
        {
            length = sprintf(text,"cmd:set_channel;set:%u;get:%u",data[1],data[2]);
        }
        break;
        case MESH_cmd_rf_chan_get:
        {
            length = sprintf(text,"cmd:get_channel;channel:%u",data[1]);
        }
        break;
        case MESH_cmd_crc_set:
        {
            length = sprintf(text,"cmd:set_crc;set:%u;get:%u",data[1],data[2]);
        }
        break;
        case MESH_cmd_crc_get:
        {
            length = sprintf(text,"cmd:get_crc;crc:%u",data[1]);
        }
        break;
        default:
        {
            length = sprintf(text,"cmd:0x%02X;resp:unknown",data[0]);
        }
        break;
    }
    return length;
}

/**
 * @brief Executes a command received in a binary format
 *
 * @param data the array starting with <cmd_id> followed by <param0><param1>,...
 * @param size the total size including the first byte of cmd_id
 */
void mesh_execute_cmd(uint8_t*data,uint8_t size,bool is_rf_request,uint8_t rf_nodeid)
{
    uint8_t resp[32];
    uint8_t resp_len;
    resp[0] = data[0];          //First byte response is always the command id requested
    switch(data[0])
    {
        case MESH_cmd_node_id_set:
        {
            //TODO persistance of parameters
            resp[1] = data[1];          //set request confirmation
            resp[2] = get_this_node_id();   //new set value as read from source after set
            resp_len = 3;
        }
        break;
        case MESH_cmd_node_id_get:
        {
            resp[1] = get_this_node_id();   //Value as read from source after set
            resp_len = 2;
        }
        break;
        case MESH_cmd_rf_chan_set:
        {
            mesh_wait_tx();//in case any action was ongoing
            nrf_esb_stop_rx();
            nrf_esb_set_rf_channel(data[1]);
            nrf_esb_start_rx();
            resp[1] = data[1];          //set channel request confirmation
            resp[2] = mesh_get_channel();   //new set value as read from source after set
            resp_len = 3;
        }
        break;
        case MESH_cmd_rf_chan_get:
        {
            resp[1] = mesh_get_channel();   //new set value as read from source after set
            resp_len = 2;
        }
        break;
        case MESH_cmd_crc_set:
        {
            mesh_wait_tx();//in case any action was ongoing
            nrf_esb_stop_rx();
            mesh_set_crc(data[1]);
            nrf_esb_start_rx();
            resp[1] = data[1];              //set crc request confirmation
            resp[2] = mesh_get_crc();       //new set value as read from source after set
            resp_len = 3;
        }
        break;
        case MESH_cmd_crc_get:
        {
            resp[1] = mesh_get_crc();       //new set value as read from source after set
            resp_len = 2;
        }
        break;
        default:
        {
            resp_len = 1;
        }
        break;
    }
    if(is_rf_request)
    {
        mesh_response_data(MESH_PID_EXE,rf_nodeid,resp,resp_len);
    }
    else
    {
        char text[128];
        uint8_t length = cmd_parse_response(text,resp,resp_len);
        m_app_cmd_handler(text,length);
    }
}

/**
 * @brief executes teh command immidiatly
 * Future extension should use a command fifo
 *
 * @param text contains a command line to execute a mesh rf function
 * supported commands :
 * * msg:0x00112233445566... note length not included as will be generated
 * where 0:control , 1:source , 2:dest/payload , 3:payload,...
 * * cmd:0x00112233445566
 * where 0x00 is the command id
 * the rest are the command parameters including : crc_cfg, header_cfg, bitrate,...
 * @param length number of characters in msg
 */
void mesh_text_request(const char*text,uint8_t length)
{
    if(strbegins(text,"msg:"))
    {
        uint8_t data[32];//TODO define global max cmd size
        uint8_t size;
        if(text2bin(text+4,length-4,data,&size))
        {
            mesh_tx_raw(data,size);
            char resp[32];
            uint8_t resp_len = sprintf(resp,"sent_msg_len:%d",size);
            m_app_cmd_handler(resp,resp_len);
        }
    }
    else if(strbegins(text,"cmd:"))
    {
        uint8_t data[32];//TODO define global max cmd size
        uint8_t size;
        if(text2bin(text+4,length-4,data,&size))
        {
            mesh_execute_cmd(data,size,false,0);//rf_nodeid unused in this case, set as 0
        }
    }
}