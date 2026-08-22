// org, not swapped
#define RGBCMD(R,G,B)  (((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3))
// RGB, bytes swapped 
#define RGB(R,G,B)  (((((G) >> 2) & 7) << 13) | (((B) >> 3) << 8)) | ((((R) >> 3) << 3) | ((G) >> 5))
// BGR
//#define RGB(R,G,B)  (((((G) >> 2) & 7) << 13) | (((R) >> 3) << 8)) | ((((B) >> 3) << 3) | ((G) >> 5))
// note that while it accepts 16-bit values, the most and least significant bytes must be swapped.
// any 16bit RGB data must be formatted as: G[LS3]B[5] | R[5]G[MS3] 

#define DRAW_LINE                       0x21
#define DRAW_RECTANGLE                  0x22
#define COPY_WINDOW                     0x23
#define DIM_WINDOW                      0x24
#define CLEAR_WINDOW                    0x25
#define FILL_WINDOW                     0x26
#define DISABLE_FILL                    0x00
#define ENABLE_FILL                     0x01
#define CONTINUOUS_SCROLLING_SETUP      0x27
#define DEACTIVE_SCROLLING              0x2E
#define ACTIVE_SCROLLING                0x2F

#define SET_COLUMN_ADDRESS              0x15
#define SET_ROW_ADDRESS                 0x75
#define SET_CONTRAST_A                  0x81
#define SET_CONTRAST_B                  0x82
#define SET_CONTRAST_C                  0x83
#define MASTER_CURRENT_CONTROL          0x87
#define SET_PRECHARGE_SPEED_A           0x8A
#define SET_PRECHARGE_SPEED_B           0x8B
#define SET_PRECHARGE_SPEED_C           0x8C
#define SET_REMAP                       0xA0
#define SET_DISPLAY_START_LINE          0xA1
#define SET_DISPLAY_OFFSET              0xA2
#define NORMAL_DISPLAY                  0xA4
#define ENTIRE_DISPLAY_ON               0xA5
#define ENTIRE_DISPLAY_OFF              0xA6
#define INVERSE_DISPLAY                 0xA7
#define SET_MULTIPLEX_RATIO             0xA8
#define DIM_MODE_SETTING                0xAB
#define SET_MASTER_CONFIGURE            0xAD
#define DIM_MODE_DISPLAY_ON             0xAC
#define DISPLAY_OFF                     0xAE
#define NORMAL_BRIGHTNESS_DISPLAY_ON    0xAF
#define POWER_SAVE_MODE                 0xB0
#define PHASE_PERIOD_ADJUSTMENT         0xB1
#define DISPLAY_CLOCK_DIV               0xB3
#define SET_GRAY_SCALE_TABLE            0xB8
#define ENABLE_LINEAR_GRAY_SCALE_TABLE  0xB9
#define SET_PRECHARGE_VOLTAGE           0xBB

#define SET_V_VOLTAGE                   0xBE

#include <math.h>
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/rtc.h"
#include "hardware/spi.h"
#include "lwip/tcp.h"
#include "font.h"
//#include "poll_log.h"

// temp sensor
#define SHT_ADDR 0x44

// oled display
#define DC_PIN 20
#define RES_PIN 21

// for analog joystick
#define X_PIN 26
#define Y_PIN 27
#define BUTTON_PIN 28

// debug/aux button
#define AUX_BUTTON_PIN 22

// WIP
#define WIFI_SSID "wifissid"
#define WIFI_PW "wifipw" 
#define WIFI_SERVER_IP "192.168.1.30"
#define SERVER_PORT 5663

#define LOG_LENGTH 100

#define CON_RETRY_DLY 500 // 50 = one second
#define CON_POLL_DLY 5 // in seconds
#define CON_TIMEOUT_DLY 12 // in seconds
#define UPL_ACK_TIMEOUT 400 // 50 = one second

#define SCRNSTATE_HOME 0
#define SCRNSTATE_GRAPH 1
#define SCRNSTATE_CNFG_MAIN 2
#define SCRNSTATE_CNFG_DISPLAY 3
#define SCRNSTATE_CNFG_TIME 4
#define SCRNSTATE_CNFG_POLL 5
#define SCRNSTATE_CONFIRM 6
#define SCRNSTATE_CNFG_REMOTE_LOG 7
#define SCRNSTATE_DEBUG_LOG 8
#define SCRNSTATE_MEM_DISPLAY 9

#define LOG_COLOR_PLAIN RGB(255, 255, 255)
#define LOG_COLOR_LWIP_STATE RGB(0, 255, 255)
#define LOG_COLOR_LWIP_CONN_ERROR RGB(255, 64, 64) 
#define LOG_COLOR_CONN_INFO RGB(0, 0, 255)
#define LOG_COLOR_LWIP_INFO RGB(0, 255, 128)
#define LOG_COLOR_GEN_ERROR RGB(255, 0, 0)
#define LOG_COLOR_GEN_DEBUG1 RGB(255, 0, 192)
#define LOG_COLOR_GEN_DEBUG2 RGB(128, 0, 192)
#define LOG_COLOR_GEN_DEBUG3 RGB(64, 0, 128)

const char* VERSION = "06B17";
#define MAX_POLLS 5000
const int GRAPH_POINTS = 79; // not intended to be modified

// up to 100
struct Msg {
    uint16_t color;
    char msg[25];
};

struct TH {
    uint16_t rawT;
    uint16_t rawH;
};

struct TH_Poll {
  // these are from 0 to 2^16, and must be converted as in Poll_TH()
  // if 0, do not accept as data
  uint16_t temp;
  uint16_t hum;

  //uint16_t minSec; // stores minutes and seconds as:
  // 0000 MMMM MMSS SSSS
  //uint8_t hour; // 00 - 23
  //uint8_t day;
  uint32_t timestamp;
};

enum CONNECTION_STATE {
    CS_OFF,
    CS_CONNECTING,
    CS_WAP_CONNECTED,
    CS_SERVER_CONNECTED
};

// wifi support is incomplete
struct Wifi_State {
    //bool prev_want_con;
    bool want_con;
    enum CONNECTION_STATE con_state;
    bool retry_server_connect; // if connected to WAP, attempt to connect until success is made
    uint16_t retry_time_delay; // time, in ticks, until retrying.
    uint16_t time_out; // time out conuter
    //enum CONNECTION_STATE prev_con_state;
    uint8_t signal_strength;
    int tcp_link_status;
    int wifi_link_status;
    uint8_t signal_strength_updly; // for polling
    uint8_t link_status_updly; // for polling
    uint8_t bars_scanning_anim; // every 30, resets @ 180

    // lwip things
    struct tcp_pcb *tcp_pcb;
    ip_addr_t addr;
};

// singleton device state.
struct State {
    bool sleeping;
    // if display is sleeping

    uint8_t current_screen;
    // current screen device is on
    // 0 = home, 1 = graph, 2 = config, 3 = display config, 4 = time config, 5 = poll config, 6 = confirmation screen, 7 = remote log config, 8 = debug log
    uint8_t back_current_screen;

    datetime_t current_time;
    // time. only displays hours and minutes, and only those can be configured

    struct TH cur_TH;

    struct Msg message_log[LOG_LENGTH];
    uint8_t msg_log_start; // for index to begin at. cannot be greater than log_len - 10 except where log_len > 10
    uint8_t msg_log_len; // does not decrease

    char* streamLog; // for printf
    uint8_t message_log_state; // 0 = regular log. 1 = stream log

    // input
    uint8_t dinput;
    // left, up, right, down
    // LURD(last) LURD(cur)
    // last, cur

    // joystick
    uint8_t button_input;
    // first two bits are last, cur

    uint8_t aux_button_input;
    // first two (LSB) bits are last, cur


    // poll delays
    uint64_t temp_poll; // when to poll in absolute time
    uint64_t sleep_poll;
    bool redraw; // used for if some information updated, and should redraw screen
    bool partial_redraw;
    uint8_t selector_delay; // used for holding down stick for faster incrementing


    // selectors
    uint8_t setting_selector; // 0-3
    uint8_t display_setting_selector; //0-1
    uint8_t poll_setting_selector; // 0-2, and 3.
    uint8_t conf_selector; // 0-1
    uint8_t remote_log_setting_selector; // 0-2
    uint8_t time_setting_selector; //0-2, and 3.
    uint8_t time_temp_hour;
    uint8_t time_temp_min;
    bool time_temp_apm; // true if AM
    uint8_t mem_sel_state; // 0-1

    // graphing 
    bool temp_graph; // if current graph displayed is temperature instead of humidity
    bool graph_select_mode;
    uint8_t graph_selector_x;
    // pointer to poll sample
    struct TH_Poll* graph_selector_sample;
    bool graph_zoom_selecting; // if zoom is currently being selected
    uint8_t zoom_selection_x; // x pos of zoom selection (start or end)

    // refer to th_log indices, inclusive
    int zoom_start_idx;
    int zoom_end_idx;
    int total_samples_drawn; // end - start, amount of samples inbetween zoom spots. in th_log indices

    int graph_points_to_draw; // between 2 and GRAPH_POINTS (amount displayed on graph in pixels)

    // settings
    uint8_t display_brightness; 
    // brightness of display
    // 0 = 100%, 1 = 50%, 2 = 25%, 3 = 13%
    uint8_t display_brightness_index; // for setting, represnted as array

    uint8_t poll_rate; 
    // polling rate for SHT41
    // 10, 15, 30, 60, 90, 120, 240 seconds
    uint8_t poll_rate_index; // for array

    uint8_t sleep_delay; 
    // delay for display power to be cut off.
    // 5, 10, 20, 30 seconds
    uint8_t sleep_delay_index; // for array

    // logging
    bool logging_enabled;
    //int total_polls; // number of logged polls
    //uint8_t memo_graph_points[GRAPH_POINTS]; // for not having to recalculate entire graph with large sample amounts. 255 = void. may not be needed
    uint16_t memo_graph_max;
    uint16_t memo_graph_min;

    // linked list. 10 TH_Polls (70/80 bytes) per node
    //struct Poll_Node th_log;

    // local log of last max_polls samples. overwrites oldest when maximum reached
    struct TH_Poll* th_log_NEW;
    // next sample spot to write. wraps around
    uint16_t th_log_next;
    // size. when 5000, start reading at th_log_next
    uint16_t th_log_size;

    // same as above. contains samples that need to be sent.
    struct TH_Poll* th_log_wifi_queue;
    uint16_t th_log_wifi_queue_size; // grows and shrinks. will generally just be the same as send_size most of the time.
    //bool wifi_log_waiting_for_ack; // NOT USED. if true, it has sent data and is waiting for the server to acknowledge. cannot send further log data if true 
    //uint16_t wifi_log_waiting_for_ack_timeout; // NOT USED. timeout on whether to abandon the request. do not delete log data if triggered, and attempt to resend.
    uint16_t th_log_wifi_queue_send_size; // the range (from 0) of data to send. this is locked with each request. exclusive, so if 1, just send index 0.
    

    // mem screen
    uint8_t mem_mx; // mouse coords
    uint8_t mem_my;
    uint8_t mem_scr; // screen number
    bool mem_viewscreen; // true if browsing screen. false if control is focused on control panel below
    uint8_t mem_curstate; // iterates through hues. 0-223

    // confirmation return-to info
    uint8_t conf_screen;
    void (*conf_fn)(void); // function to call when 'yes' is selected

    // waits for the internal absolute time to be within 20ms of the time to sync to
    bool sync_time;
    uint8_t sync_hour;
    uint8_t sync_min;
    uint8_t sync_sec;

    // wifi
    struct Wifi_State wifi_state;
};

static const char* cyw43_tcpip_link_status_name(int status)
{
    switch (status) {
    case CYW43_LINK_DOWN:
        return "link down";
    case CYW43_LINK_JOIN:
        return "joining";
    case CYW43_LINK_NOIP:
        return "no ip";
    case CYW43_LINK_UP:
        return "link up";
    case CYW43_LINK_FAIL:
        return "link fail";
    case CYW43_LINK_NONET:
        return "network fail";
    case CYW43_LINK_BADAUTH:
        return "bad auth";
    }
    return "unknown";
}

static const char* lwip_err_name(err_t err) {
    switch (err) {
        /** No error, everything OK. */
        case ERR_OK: return "ok";
        /** Out of memory error.     */
        case ERR_MEM: return "mem";
        /** Buffer error.            */
        case ERR_BUF: return "buf";
        /** Timeout.                 */
        case ERR_TIMEOUT: return "timeout";
        /** Routing problem.         */
        case ERR_RTE: return "rte";
        /** Operation in progress    */
        case ERR_INPROGRESS: return "inprog";
        /** Illegal value.           */
        case ERR_VAL: return "val";
        /** Operation would block.   */
        case ERR_WOULDBLOCK: return "wblock";
        /** Address in use.          */
        case ERR_USE: return "use";
        /** Already connecting.      */
        case ERR_ALREADY: return "already";
        /** Conn already established.*/
        case ERR_ISCONN: return "iscon";
        /** Not connected.           */
        case ERR_CONN: return "isntcon";
        /** Low-level netif error    */
        case ERR_IF: return "if";
        /** Connection aborted.      */
        case ERR_ABRT: return "abrt";
        /** Connection reset.        */
        case ERR_RST: return "rst";
        /** Connection closed.       */
        case ERR_CLSD: return "clsd";
        /** Illegal argument.        */
        case ERR_ARG: return "arg";

        default: return "???";
    }
}

void draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color);
void draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t outlineColor, uint16_t fillColor);
void draw_point(uint8_t x, uint8_t y, uint16_t color);

void add_log_msg(struct State* device_state, const char* msg, uint16_t color);

void send_log_data(struct State* device_state);
void send_setting_sync(struct State* device_state);
void send_data(struct State* device_state, uint8_t* data, uint16_t dataLen, uint8_t pIdent, bool immediate);
err_t server_connected_cb(void* arg, struct tcp_pcb* pcb, err_t err);
void init_client(struct State* device_state);
err_t close_connection(void* arg);
void init_wifi(struct State* device_state);
void turn_wifi_off(struct State* device_state);

datetime_t get_next_minute(const datetime_t* dt);
void reset_temp_log();

float rawT_to_float(uint16_t t) {
    return (315 * ((float)t / 65535)) - 49;
}

float rawH_to_float(uint16_t h) {
    return (125 * ((float)h / 65535)) - 6;
}

// in fahrenheit. best to call not any more frequently than every 10 seconds per manufacturer
struct TH poll_TH() {
    uint8_t command = 0xFD; 
    uint8_t res[6];
    i2c_write_blocking(i2c0, SHT_ADDR, &command, 1, false);
    sleep_ms(10);
    i2c_read_blocking(i2c0, SHT_ADDR, (uint8_t*)(&res), 6, false);

    uint16_t Hdata = 0;
    uint16_t Tdata = 0;

    // Tdata[2] CRC[1] Hdata[2] CRC[1]

    Tdata = (res[0] * 0x100) + res[1];
    Hdata = (res[3] * 0x100) + res[4];

    struct TH data;
    //data.temp = (315 * ((float)Tdata / 65535)) - 49;
    //data.hum = (125 * ((float)Hdata / 65535)) - 6;
    data.rawT = Tdata;
    data.rawH = Hdata;

    //if (data.hum > 100) data.hum = 100.0f;
    //if (data.hum < 0) data.hum = 0.0f;

    return data;
}

// 96x64
// 96 columns, 64 rows
void command_display(uint8_t cmd) {
    // set for interpreataion as commmand, otherwise will see as display data (if 1)
    gpio_put(DC_PIN, 0);
    // write
    spi_write_blocking(spi_default, &cmd, 1);
}

void init_display() {
    // RES (GP21) - reset signal input. keep high during 'normal operation'. when low, chip is initialized
    // DC (GP20) - when high, data from D[15:0] will be interpreted as display data. when low, D[15:0] will be interpreted as command
    
    // initialize via RESet pin
    sleep_ms(10);
    gpio_put(RES_PIN, 0);
    sleep_ms(10);
    gpio_put(RES_PIN, 1);

    command_display(DISPLAY_OFF);              //Display Off
    command_display(SET_CONTRAST_A);           //Set contrast for color A
    command_display(0xFF);                     //145 0x91
    command_display(SET_CONTRAST_B);           //Set contrast for color B
    command_display(0xFF);                     //80 0x50
    command_display(SET_CONTRAST_C);           //Set contrast for color C
    command_display(0xFF);                     //125 0x7D
    command_display(MASTER_CURRENT_CONTROL);   //master current control
    command_display(0x06);                     //6
    command_display(SET_PRECHARGE_SPEED_A);    //Set Second Pre-change Speed For ColorA
    command_display(0x64);                     //100
    command_display(SET_PRECHARGE_SPEED_B);    //Set Second Pre-change Speed For ColorB
    command_display(0x78);                     //120
    command_display(SET_PRECHARGE_SPEED_C);    //Set Second Pre-change Speed For ColorC
    command_display(0x64);                     //100
    command_display(SET_REMAP);                //set remap & data format
    command_display(0x72);                     //0x72              
    command_display(SET_DISPLAY_START_LINE);   //Set display Start Line
    command_display(0x0);
    command_display(SET_DISPLAY_OFFSET);       //Set display offset
    command_display(0x0);
    command_display(NORMAL_DISPLAY);           //Set display mode
    command_display(SET_MULTIPLEX_RATIO);      //Set multiplex ratio
    command_display(0x3F);                     
    command_display(SET_MASTER_CONFIGURE);     //Set master configuration
    command_display(0x8E);                     
    command_display(POWER_SAVE_MODE);          //Set Power Save Mode
    command_display(0x00);                     //0x00
    command_display(PHASE_PERIOD_ADJUSTMENT);  //phase 1 and 2 period adjustment
    command_display(0x31);                     //0x31
    command_display(DISPLAY_CLOCK_DIV);        //display clock divider/oscillator frequency
    command_display(0xF0);
    command_display(SET_PRECHARGE_VOLTAGE);    //Set Pre-Change Level
    command_display(0x3A);
    command_display(SET_V_VOLTAGE);            //Set vcomH
    command_display(0x3E);                    
    command_display(DEACTIVE_SCROLLING);       //disable scrolling
    command_display(FILL_WINDOW); // turn rect filling on, required for the clear screen command (which i guess just calls draw_rectangle anyway)
    command_display(ENABLE_FILL);
    command_display(NORMAL_BRIGHTNESS_DISPLAY_ON);    //set display on  
}

// needs rect filling enabled
void clear_screen() {
    command_display(CLEAR_WINDOW);
    command_display(0x00);
    command_display(0x00);
    
    command_display(0x5F);
    command_display(0x3F);
}

void init_oled_spi() {
    // init SSD1331 (OLED) via spi @ 2mhz
    spi_init(spi_default, 1000 * 2000);
    
    gpio_set_function(PICO_DEFAULT_SPI_SCK_PIN, GPIO_FUNC_SPI);
    gpio_set_function(PICO_DEFAULT_SPI_TX_PIN, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PICO_DEFAULT_SPI_CSN_PIN);
    gpio_set_dir(PICO_DEFAULT_SPI_CSN_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 1);

    gpio_init(DC_PIN);
    gpio_set_dir(DC_PIN, GPIO_OUT);
    gpio_put(DC_PIN, 0);
    gpio_init(RES_PIN);
    gpio_set_dir(RES_PIN, GPIO_OUT);
    gpio_put(RES_PIN, 1);

    // if set high, oled will be turned off (at least from receiving data, i'd guess)
    gpio_put(PICO_DEFAULT_SPI_CSN_PIN, 0);

    init_display();
    clear_screen();
    sleep_ms(1);
}

// does not cut power, so DRAM is saved
void display_off() {
    command_display(DISPLAY_OFF);
}

void display_on(bool dim) {
    command_display((dim) ? DIM_MODE_DISPLAY_ON : NORMAL_BRIGHTNESS_DISPLAY_ON);
}

void set_display_brightness(uint8_t power) {
    command_display(DIM_MODE_SETTING);
    command_display(0x00); // does nothing, required
    command_display(power);
    command_display(power);
    command_display(power);
    command_display(0x0F); // something '-precharge-', i do not know
    command_display(DIM_MODE_DISPLAY_ON);
}

void send_color_command(uint16_t color) {
    // red (5-bits)
    // backshift is because it wants the LS bit to be zero for red and blue (both 5-bit)
    command_display((uint8_t)((color >> 11) << 1));
    // green (6-bits)
    command_display((uint8_t)((color >> 5) % 0x40));
    // blue (5-bits)
    command_display((uint8_t)((color % 0x20) << 1));
}

void draw_line(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t color) {
    command_display(DRAW_LINE);
    command_display(x1);
    command_display(y1);
    command_display(x2);
    command_display(y2);
    send_color_command(color);
}

void draw_rectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, uint16_t outlineColor, uint16_t fillColor) {
    command_display(DRAW_RECTANGLE);
    command_display(x1);
    command_display(y1);
    command_display(x2);
    command_display(y2);
    send_color_command(outlineColor);
    send_color_command(fillColor);
}

void write_display_data(uint8_t startX, uint8_t endX, uint8_t startY, uint8_t endY, const uint8_t* data, size_t len) {
    command_display(SET_COLUMN_ADDRESS);
    command_display(startX);
    command_display(endX);
    command_display(SET_ROW_ADDRESS);
    command_display(startY);
    command_display(endY);
    
    // DC_PIN 1 = display data. DC_PIN 0 = command data.
    gpio_put(DC_PIN, 1);
    spi_write_blocking(spi_default, data, len);
}

void read_display_data(uint8_t* data) {
    gpio_put(DC_PIN, 1);

    // "Also, a dummy read is required before the first data read."
    spi_read_blocking(spi_default, 0, data, 64 * 96 * 2);
}

void draw_point(uint8_t x, uint8_t y, uint16_t color) {
    write_display_data(x, x + 1, y, y + 1, (uint8_t*)(&color), 2);
}

void draw_font5x3(const char* text, uint8_t x, uint8_t y, uint16_t color, uint16_t backColor) {

    int cx = x;

    // TODO: see if entire word can be done in single write?

    for (size_t t = 0; t < strlen(text); t++) {

        char c = text[t];
        bool lastChar = (t == strlen(text) - 1);
        
        int pos = (int)(c) - 0x20;
        // out of bounds, print missing character 
        if (pos > 0x5A || (pos > 0x3A && pos < 0x41) || pos < 0x00) pos = 4;

        // support lowercase (converts to uppercase)
        if (pos < 0x5B && pos > 0x40) pos -= 0x20;

        // change color of letter
        uint16_t glyph[15];
        uint16_t spacing[5];
        uint16_t period[5]; // only used if pos == 12
        for (int i = 0; i < 15; i++) {
            glyph[i] = Font3x5[pos][i];

            if (glyph[i] == 0xFFFF) glyph[i] = color;
            else glyph[i] = backColor;
        }
        // add backcolor for spacing
        for (int i = 0; i < 5; i++) {
            spacing[i] = backColor;
            period[i] = backColor;
        }

        int spacingWidth = (pos == 12) ? 1 : 3;
        
        // draw thin '.' (only 1x5 instead of 3x5)
        if (pos == 12) period[4] = color;

        if (pos == 12) write_display_data(cx, cx, y, y + 4, (const uint8_t*)period, 10);
        else write_display_data(cx, cx + 2, y, y + 4, (const uint8_t*)glyph, 30);
        if (!lastChar) write_display_data(cx + spacingWidth, cx + spacingWidth, y, y + 4, (const uint8_t*)spacing, 10);

        cx += (pos == 12) ? 2 : 4;
    }
}

void draw_font10x13(const char* text, uint8_t x, uint8_t y, uint16_t color) {
    int cx = x;

    for (size_t t = 0; t < strlen(text); t++) {

        char c = text[t];
        
        int pos = (int)(c) - 0x20;
        if (pos < 0x00) continue;

        // ascii to index for non-numbers
        switch (pos) {
            case 0x03: pos = 11; break; // deg (actually '#' as text, rendered as degree)
            case 0x26: pos = 12; break; // F
            case 0x28: pos = 13; break; // H
            case 0x05: pos = 14; break; // %
            case 0x0E: pos = 15; break; // .
            case 0x34: pos = 16; break; // T
            case 0x21: pos = 17; break; // A
            case 0x2D: pos = 18; break; // M
            case 0x30: pos = 19; break; // P
            case 0x00: pos = 20; break; // space
            case 0x32: pos = 21; break; // R
            default: pos -= 0x10; break; // is number or colon
        }

        if (pos > 21 || pos < 0) continue;

        // change color of letter
        // font.h and RGB (macro function) are formatted as:
        // MSB: G[LS3]B[5]
        // LSB: R[5]G[MS3]
        // color is formatted as RGB
        uint16_t glyph[130];
        for (int i = 0; i < 130; i++) {
            glyph[i] = Font10x13[pos][i];
            
            uint8_t blue = (glyph[i] >> 8) & 31;
            uint8_t green = ((glyph[i] & 7) << 3) | (glyph[i] >> 13);
            uint8_t red = (glyph[i] >> 3) & 31;
            uint8_t newBlue = (color >> 8) & 31;
            uint8_t newGreen = ((color & 7) << 3) | (color >> 13);
            uint8_t newRed = (color >> 3) & 31;

            uint8_t finalBlue = ((blue / 32.0f) * (newBlue / 32.0f)) * 32.0f;
            uint8_t finalGreen = ((green / 64.0f) * (newGreen / 64.0f)) * 64.0f;
            uint8_t finalRed = ((red / 32.0f) * (newRed / 32.0f)) * 32.0f;

            glyph[i] = RGB(finalRed * 8, finalGreen * 4, finalBlue * 8);
        }

        write_display_data(cx, cx + 9, y, y + 12, (const uint8_t*)glyph, 260);
        cx += 10;
    }
}

void draw_config() {
    write_display_data(19, 76, 2, 14, (const uint8_t*)ImgConfig, 1508);
}

void draw_arrow(uint8_t x, uint8_t y, bool down, bool blue) {
    write_display_data(x, x + 9, y, y + 5, (const uint8_t*)ImgArrows[(down << 1) | blue], 120);
}

void draw_mini_arrow(uint8_t x, uint8_t y, bool right, bool blue) {
    write_display_data(x, x + 5, y, y + 4, (const uint8_t*)MiniArrows[(right << 1) | blue], 60);
}

void draw_x_axis(uint8_t y, float n) {
    uint16_t lineColor = RGBCMD(32, 32, 32);

    if (n >= 200.0) n = 199.9;
    if (n <= 0) n = 0;

    // for values with 1, 2, 3 digits
    int spacing = 8;
    if (n < 100 && n >= 10) spacing = 12;
    if (n > 99) spacing = 16; 
    draw_line(0, y, 95, y, lineColor);
    sleep_us(1);
    draw_line(91 - MIN(spacing, 14), y + 1, 95, y + 1, lineColor);
    sleep_us(1);
    draw_line(92 - MIN(spacing, 14), y + 2, 95, y + 2, lineColor);
    sleep_us(1);
    draw_line(93 - MIN(spacing, 14), y + 3, 95, y + 3, lineColor);
    sleep_us(1);
    draw_line(94 - MIN(spacing, 14), y + 4, 95, y + 4, lineColor);
    sleep_us(1);

    char nstr[6];
    sprintf(nstr, "%.1f", n);

    uint8_t dotIdx = 0;
    if (n < 100 && n >= 10) dotIdx = 2;
    else if (n >= 100) dotIdx = 3;
    else if (n < 10) dotIdx = 1;
    nstr[dotIdx] = ',';

    draw_font5x3(nstr, 95 - spacing, y, 0xFFFF, RGB(32, 32, 32));
}

void draw_wifi_bars(struct State* device_state, uint8_t x, uint8_t y) {
    // off [gray]
    // unconnected (to WAP) [red one point]
    // unconnected (to server) [yellow]
    // connected [green]

    uint16_t color;
    uint16_t gray = RGBCMD(128, 128, 128);
    int con_status = device_state->wifi_state.con_state;
    switch (con_status) {
        case CS_OFF: color = gray; break;
        case CS_CONNECTING: color = RGBCMD(255, 0, 0); break;
        case CS_WAP_CONNECTED: color = RGBCMD(255, 255, 0); break;
        case CS_SERVER_CONNECTED: color = RGBCMD(0, 255, 0); break;
    }

    if (con_status == CS_CONNECTING) {
        uint8_t costume = device_state->wifi_state.bars_scanning_anim / 30;
        draw_line(x, y + 3, x, y + 3, (costume == 0) ? color : gray); // "line"
        draw_line(x + 2, y + 2, x + 2, y + 3, (costume == 1 || costume == 5) ? color : gray);
        draw_line(x + 4, y + 1, x + 4, y + 3, (costume == 2 || costume == 4) ? color : gray);
        draw_line(x + 6, y, x + 6, y + 3, (costume == 3) ? color : gray);
    } else {
        uint8_t s = device_state->wifi_state.signal_strength;
        draw_line(x, y + 3, x, y + 3, color); // "line"
        draw_line(x + 2, y + 2, x + 2, y + 3, (s >= 64) ? color : gray);
        draw_line(x + 4, y + 1, x + 4, y + 3, (s >= 128) ? color : gray);
        draw_line(x + 6, y, x + 6, y + 3, (s >= 192) ? color : gray);
    }
    
}

uint8_t getHour(const datetime_t* t) {
    uint8_t hour = t->hour % 12;
    if (hour == 0) hour = 12;
    return hour;
}

void getNormalTime(struct State* device_state, uint32_t curTime, uint32_t abSeconds, uint8_t* setDay, uint8_t* setHour, uint8_t* setMinute, uint8_t* setSecond) {
    // this function will sometimes return a result off by one second, 1/60 chance

    rtc_get_datetime(&device_state->current_time);
    // don't allow future dates
    if (curTime < abSeconds) {
        if (setDay != NULL) *setDay = device_state->current_time.day;
        if (setHour != NULL) *setHour = device_state->current_time.hour;
        if (setMinute != NULL) *setMinute = device_state->current_time.min;
        if (setSecond != NULL) *setSecond = device_state->current_time.sec;
        return;
    }
    uint32_t diff = curTime - abSeconds;

    uint8_t dayDiff = diff / 86400;
    diff %= 86400;
    uint8_t hourDiff = diff / 3600;
    diff %= 3600;
    uint8_t minDiff = diff / 60;
    diff %= 60;
    uint8_t secDiff = diff;

    uint8_t finalSec, finalMin, finalHour, finalDay;

    // handle carries
    if (device_state->current_time.sec < secDiff) {
        minDiff++;
        finalSec = 60 - (secDiff - device_state->current_time.sec);
    } else {
        finalSec = device_state->current_time.sec - secDiff;
    }

    if (device_state->current_time.min < minDiff) {
        hourDiff++;
        finalMin = 60 - (minDiff - device_state->current_time.min);
    } else {
        finalMin = device_state->current_time.min - minDiff;
    }

    if (device_state->current_time.hour < hourDiff) {
        dayDiff++;
        finalHour = 24 - (hourDiff - device_state->current_time.hour);
    } else {
        finalHour = device_state->current_time.hour - hourDiff;
    }

    finalDay = device_state->current_time.day - dayDiff;

    if (setDay != NULL) *setDay = finalDay;
    if (setHour != NULL) *setHour = finalHour;
    if (setMinute != NULL) *setMinute = finalMin;
    if (setSecond != NULL) *setSecond = finalSec;
}

void read_input(struct State* device_state) {
    // set previous readings to 'last' position
    device_state->dinput <<= 4;
    device_state->button_input <<= 1;
    device_state->aux_button_input <<= 1;
    float deadzone = 0.75;

    // read X axis
    adc_select_input(0);
    float xRaw = adc_read();
    float xPos = xRaw / 4096;
    if (xPos < 1.0f - deadzone) {
        // right
        device_state->dinput |= 2;
    } else if (xPos > deadzone) {
        // left
        device_state->dinput |= 8;
    }

    // read Y axis
    adc_select_input(1);
    float yRaw = adc_read();
    float yPos = yRaw / 4096;
    if (yPos < 1.0f - deadzone) {
        // up
        device_state->dinput |= 4;
    } else if (yPos > deadzone) {
        // down
        device_state->dinput |= 1;
    }

    // read... button
    adc_select_input(2);
    float bRaw = adc_read();
    float bPos = bRaw / 4096;
    if (bPos > 0.8f) device_state->button_input |= 1;

    // aux, non-joystick button
    if (gpio_get(AUX_BUTTON_PIN)) device_state->aux_button_input |= 1;
}

void handle_input(struct State* device_state, bool remote) {

    // reset sleep delay
    if ((device_state->button_input > 0 || device_state->dinput > 0) && !remote) {
        device_state->sleep_poll = get_absolute_time() + (device_state->sleep_delay * 1000000); 
    }

    // if not sleeping
    if (!device_state->sleeping && !remote) {
        bool syncSettings = false;

        bool leftNew = ((device_state->dinput & 8) > 0) && ((device_state->dinput & 128)) == 0;
        bool upNew = ((device_state->dinput & 4) > 0) && ((device_state->dinput & 64)) == 0;
        bool rightNew = ((device_state->dinput & 2) > 0) && ((device_state->dinput & 32)) == 0;
        bool downNew = ((device_state->dinput & 1) > 0) && ((device_state->dinput & 16)) == 0;
        bool buttonNew = ((device_state->button_input & 1) > 0) && ((device_state->button_input & 2)) == 0;
        bool auxButtonNew = ((device_state->aux_button_input & 1) > 0) && ((device_state->aux_button_input & 2)) == 0;

        bool upCur = (device_state->dinput & 4) > 0;
        bool downCur = (device_state->dinput & 1) > 0;
        bool leftCur = (device_state->dinput & 8) > 0;
        bool rightCur = (device_state->dinput & 2) > 0;

        uint8_t prevScreen = device_state->current_screen;

        if (device_state->current_screen < SCRNSTATE_CNFG_DISPLAY && !(device_state->current_screen == SCRNSTATE_GRAPH && device_state->graph_select_mode)) {
            if (rightNew) device_state->current_screen++;
            if (leftNew) (device_state->current_screen == SCRNSTATE_HOME) ? device_state->current_screen = SCRNSTATE_CNFG_MAIN : device_state->current_screen--;
            device_state->current_screen %= 3;
            if (rightNew || leftNew) device_state->redraw = true;
        }

        // newly switched to screen
        if (prevScreen != device_state->current_screen) {
            switch (device_state->current_screen) {
                case SCRNSTATE_GRAPH:
                {
                    // prevents odd behavior when crossing this threshold
                    if (device_state->th_log_size < 2) break;

                    device_state->zoom_start_idx = 0;
                    device_state->zoom_end_idx = device_state->th_log_size - 1; 
                    device_state->total_samples_drawn = device_state->th_log_size;

                    if (device_state->total_samples_drawn > GRAPH_POINTS) device_state->graph_points_to_draw = GRAPH_POINTS;
                    else device_state->graph_points_to_draw = device_state->total_samples_drawn;

                    device_state->graph_zoom_selecting = false;
                    // not needed probably
                    device_state->zoom_selection_x = device_state->graph_selector_x;
                } break;
            }
        }

        if (device_state->current_screen == SCRNSTATE_CNFG_DISPLAY || device_state->current_screen == SCRNSTATE_CNFG_POLL || device_state->current_screen == SCRNSTATE_CNFG_REMOTE_LOG) {
            if (rightNew || leftNew) {
                device_state->current_screen = SCRNSTATE_CNFG_MAIN;
                device_state->redraw = true;
            }
        }

        if (device_state->current_screen == SCRNSTATE_CNFG_TIME || device_state->current_screen == SCRNSTATE_DEBUG_LOG) {
            if (upCur || downCur) {
                device_state->selector_delay++;
            } else device_state->selector_delay = 0;
            if (device_state->selector_delay > 254) device_state->selector_delay = 31;
        }

        if (device_state->current_screen == SCRNSTATE_GRAPH && device_state->graph_select_mode) {
            if (leftCur || rightCur) {
                device_state->selector_delay++;
            } else device_state->selector_delay = 0;
            if (device_state->selector_delay > 254) device_state->selector_delay = 31;
        }

        if (device_state->current_screen == SCRNSTATE_MEM_DISPLAY && device_state->mem_viewscreen) {
            if (leftCur || rightCur || upCur || downCur) {
                device_state->selector_delay++;
            } else device_state->selector_delay = 0;
            if (device_state->selector_delay > 254) device_state->selector_delay = 31;
        }

        if (auxButtonNew) {
            if (device_state->current_screen == SCRNSTATE_DEBUG_LOG) {
                device_state->current_screen = device_state->back_current_screen;
                device_state->redraw = true;
            } else {
                device_state->back_current_screen = device_state->current_screen;
                device_state->current_screen = SCRNSTATE_DEBUG_LOG;
                device_state->redraw = true;
            }
        }

        switch (device_state->current_screen) {
            case SCRNSTATE_HOME: {
                
            } break;
            case SCRNSTATE_GRAPH: {
                // prevent needless refreshes
                if (device_state->th_log_size < 2) break;

                if (device_state->graph_select_mode) {
                    if (buttonNew) {
                        // not zoom selecting
                        if (!device_state->graph_zoom_selecting) {
                            device_state->graph_zoom_selecting = true;
                            device_state->zoom_selection_x = device_state->graph_selector_x;
                        } else {
                            // already zoom selecting, submit changes
                            device_state->graph_zoom_selecting = false;

                            // cancel if selector is in same spot
                            if (device_state->zoom_selection_x != device_state->graph_selector_x) {
                                int oldStart = device_state->zoom_start_idx;
                                
                                device_state->zoom_start_idx = oldStart + round((MIN(device_state->zoom_selection_x, device_state->graph_selector_x) / (double)(device_state->graph_points_to_draw - 1)) * (device_state->total_samples_drawn - 1));
                                device_state->zoom_end_idx = oldStart + round((MAX(device_state->zoom_selection_x, device_state->graph_selector_x) / (double)(device_state->graph_points_to_draw - 1)) * (device_state->total_samples_drawn - 1));

                                device_state->total_samples_drawn = (device_state->zoom_end_idx + 1) - device_state->zoom_start_idx;
                                if (device_state->total_samples_drawn > GRAPH_POINTS) device_state->graph_points_to_draw = GRAPH_POINTS;
                                else device_state->graph_points_to_draw = device_state->total_samples_drawn;

                                device_state->graph_selector_x = 0;
                            }
                        }
                        device_state->redraw = true;
                    } else if (downNew) {
                        device_state->graph_select_mode = false;
                        device_state->graph_zoom_selecting = false;
                        device_state->redraw = true;
                    } else if (rightNew || leftNew || (device_state->selector_delay > 30 && device_state->selector_delay % 2 == 0)) {
                        if (leftCur) {
                            device_state->graph_selector_x--;
                            if (device_state->graph_selector_x == 255) device_state->graph_selector_x = device_state->graph_points_to_draw - 1;
                            device_state->partial_redraw = true;
                        }
                        if (rightCur) {
                            device_state->graph_selector_x++;
                            if (device_state->graph_selector_x >= (uint8_t)device_state->graph_points_to_draw) device_state->graph_selector_x = 0;
                            device_state->partial_redraw = true;
                        }
                    }
                } else {
                    if (buttonNew) {
                        device_state->temp_graph = !device_state->temp_graph;
                        device_state->redraw = true;
                    }
                    if (upNew && device_state->th_log_size >= 2) {
                        device_state->graph_select_mode = true;
                        device_state->selector_delay = 0;
                        device_state->redraw = true;
                    }
                }
                
            } break;
            case SCRNSTATE_CNFG_MAIN: {
                if (downNew) device_state->setting_selector++;
                if (upNew) (device_state->setting_selector == 0) ? device_state->setting_selector = 4 : device_state->setting_selector--;
                device_state->setting_selector %= 5;
                if (downNew || upNew) device_state->redraw = true;

                if (buttonNew) {
                    switch (device_state->setting_selector) {
                        // go to time config
                        case 0: {
                            rtc_get_datetime(&device_state->current_time);
                            device_state->current_screen = SCRNSTATE_CNFG_TIME;
                            device_state->time_temp_hour = getHour(&device_state->current_time);
                            device_state->time_temp_min = device_state->current_time.min;
                            device_state->time_temp_apm = (device_state->current_time.hour < 12);
                            device_state->time_setting_selector = 3;
                            device_state->redraw = true;
                        } break;

                        // go to display config
                        case 1: {
                            device_state->current_screen = SCRNSTATE_CNFG_DISPLAY;
                            device_state->display_setting_selector = 0;
                            device_state->redraw = true;
                        } break;

                        // go to poll config
                        case 2: {
                            device_state->current_screen = SCRNSTATE_CNFG_POLL;
                            device_state->poll_setting_selector = 0;
                            device_state->redraw = true;
                        } break;

                        // go to remote log config
                        case 3: {
                            device_state->current_screen = SCRNSTATE_CNFG_REMOTE_LOG;
                            device_state->remote_log_setting_selector = 0;
                            device_state->redraw = true;
                        } break;

                        // go to memory inspector
                        case 4: {
                            device_state->current_screen = SCRNSTATE_MEM_DISPLAY;
                            device_state->redraw = true;
                        } break;
                    }
                }
            } break;
            case SCRNSTATE_CNFG_DISPLAY: {
                if (downNew) device_state->display_setting_selector++;
                if (upNew) (device_state->display_setting_selector == 0) ? device_state->display_setting_selector = 2 : device_state->display_setting_selector--;
                device_state->display_setting_selector %= 3;
                if (downNew || upNew) device_state->redraw = true;

                if (buttonNew) {
                    switch (device_state->display_setting_selector) {
                        case 0: {
                            uint8_t brightnesses[4] = { 13, 25, 50, 100 };
                            device_state->display_brightness_index++;
                            device_state->display_brightness_index %= 4;
                            device_state->display_brightness = brightnesses[device_state->display_brightness_index];
                            device_state->redraw = true; // not partial_redraw; set_display_brightness appears to clear the display
                            set_display_brightness(device_state->display_brightness * 2.55);
                            sleep_ms(1); // seems to redraw the screen when it is toggled to/from dim mode
                            syncSettings = true;
                        } break;
                        case 1: {
                            uint8_t delays[4] = { 5, 10, 20, 30 };
                            device_state->sleep_delay_index++;
                            device_state->sleep_delay_index %= 4;
                            device_state->sleep_delay = delays[device_state->sleep_delay_index];
                            device_state->partial_redraw = true; 
                            syncSettings = true;
                        } break;
                        case 2: {
                            device_state->current_screen = SCRNSTATE_CNFG_MAIN;
                            device_state->redraw = true;
                        } break;
                    }
                }
                
            } break;
            case SCRNSTATE_CNFG_TIME: {
                // if in time setting area
                if (device_state->time_setting_selector < 3) {
                    // horizontal movement
                    if (rightNew) device_state->time_setting_selector++;
                    device_state->time_setting_selector %= 3;

                    if (device_state->time_setting_selector == 0 && leftNew) device_state->time_setting_selector = 2;
                    else if (leftNew) device_state->time_setting_selector--;
                    
                    if (rightNew || leftNew) device_state->redraw = true;

                    // vertical movement (actual configuration)
                    if ((upNew || downNew) || (device_state->selector_delay > 30 && device_state->selector_delay % 3 == 0)) {
                        switch (device_state->time_setting_selector) {
                            // hour
                            case 0: {
                                device_state->time_temp_hour += (upCur) ? 1 : -1;
                                if (device_state->time_temp_hour == 13) device_state->time_temp_hour = 1;
                                if (device_state->time_temp_hour == 0) device_state->time_temp_hour = 12;
                                device_state->partial_redraw = true;
                            } break;
                            // min
                            case 1: {
                                if (device_state->time_temp_min == 59 && upCur) device_state->time_temp_min = 0;
                                else if (device_state->time_temp_min == 0 && downCur) device_state->time_temp_min = 59;
                                else device_state->time_temp_min += (upCur) ? 1 : -1;
                                device_state->partial_redraw = true;
                            } break;
                            // AM/PM
                            case 2: {
                                device_state->time_temp_apm = !device_state->time_temp_apm;
                                device_state->partial_redraw = true;
                            } break;
                        }
                    }

                    // leave
                    if (buttonNew) { 
                        device_state->time_setting_selector = 3;
                        device_state->redraw = true;
                    }
                } else {
                    if (upNew) { 
                        device_state->time_setting_selector = 0;
                        device_state->selector_delay = 0;
                        device_state->redraw = true;
                    }
                    else if (rightNew || leftNew || buttonNew) {
                        device_state->current_screen = SCRNSTATE_CNFG_MAIN;
                        device_state->redraw = true;
                        // apply changes
                        if (buttonNew) {
                            device_state->current_time.hour = (device_state->time_temp_hour == 12 ? 0 : device_state->time_temp_hour) + (device_state->time_temp_apm ? 0 : 12);
                            device_state->current_time.min = device_state->time_temp_min;
                            rtc_set_datetime(&device_state->current_time);
                            char msg[25];
                            sprintf(msg, "TIME SET: %.2i;%.2i;%.2i", device_state->time_temp_hour, device_state->time_temp_min, device_state->current_time.sec);
                            add_log_msg(device_state, msg, LOG_COLOR_PLAIN);
                        }
                    }
                }
            } break;

            // poll settings
            case SCRNSTATE_CNFG_POLL: {
                if (downNew) device_state->poll_setting_selector++;
                if (upNew) (device_state->poll_setting_selector == 0) ? device_state->poll_setting_selector = 3 : device_state->poll_setting_selector--;
                device_state->poll_setting_selector %= 4;
                if (downNew || upNew) device_state->redraw = true;

                if (buttonNew) {
                    switch (device_state->poll_setting_selector) {
                        // logging on/off
                        case 0: {
                            device_state->logging_enabled = !device_state->logging_enabled;
                            device_state->partial_redraw = true;
                            syncSettings = true;
                        } break;
                        // poll rate
                        case 1: {
                            uint8_t pollrates[7] = { 10, 15, 30, 60, 90, 120, 240 };
                            device_state->poll_rate_index++;
                            device_state->poll_rate_index %= 7;
                            device_state->poll_rate = pollrates[device_state->poll_rate_index];
                            // reset next poll to new rate
                            device_state->temp_poll = get_absolute_time() + (device_state->poll_rate * 1000000); 
                            device_state->partial_redraw = true;
                            syncSettings = true;
                        } break;
                        // clear log
                        case 2: {
                            device_state->current_screen = SCRNSTATE_CONFIRM;
                            device_state->redraw = true;
                            device_state->conf_fn = reset_temp_log;
                            device_state->conf_screen = 5;
                            device_state->conf_selector = 0;
                        } break;
                        // back
                        case 3: {
                            device_state->current_screen = SCRNSTATE_CNFG_MAIN;
                            device_state->redraw = true;
                        } break;
                    }
                }
            } break;
            case SCRNSTATE_CONFIRM: {
                if (downNew) {
                    device_state->current_screen = device_state->conf_screen;
                    device_state->redraw = true;
                } else {
                    if (leftNew && device_state->conf_selector == 1) {
                        device_state->conf_selector = 0;
                        device_state->partial_redraw = true;
                    } else if (rightNew && device_state->conf_selector == 0) {
                        device_state->conf_selector = 1;
                        device_state->partial_redraw = true;
                    }

                    if (buttonNew) {
                        device_state->current_screen = device_state->conf_screen;
                        device_state->redraw = true;
                        if (device_state->conf_selector == 1) {
                            device_state->conf_fn();
                        } 
                    }
                }
            } break;
            case SCRNSTATE_CNFG_REMOTE_LOG: {
                if (downNew) device_state->remote_log_setting_selector++;
                if (upNew) (device_state->remote_log_setting_selector == 0) ? device_state->remote_log_setting_selector = 2 : device_state->remote_log_setting_selector--;
                device_state->remote_log_setting_selector %= 3;
                if (downNew || upNew) device_state->redraw = true;

                switch (device_state->remote_log_setting_selector) {
                    // on/off
                    case 0: {
                        if (buttonNew) {
                            // turn off
                            if (device_state->wifi_state.want_con) {
                                turn_wifi_off(device_state);

                                // turn on
                            } else {
                                init_wifi(device_state);
                            }
                            device_state->wifi_state.want_con = !device_state->wifi_state.want_con;
                            device_state->partial_redraw = true;
                        }
                        break;
                    }
                    // debug poll
                    case 1: {
                        if (buttonNew && device_state->wifi_state.con_state == CS_SERVER_CONNECTED) {
                            send_data(device_state, "testpacket", 11, '_', true);
                            device_state->partial_redraw = true;
                        }
                    } break;
                    // back
                    case 2: {
                        if (buttonNew) {
                            device_state->current_screen = SCRNSTATE_CNFG_MAIN;
                            device_state->redraw = true;
                        }
                        break;
                    }
                }
                
            } break;
            case SCRNSTATE_DEBUG_LOG: {
                // debug
                if (buttonNew) {
                    if (device_state->message_log_state == 0) add_log_msg(device_state, "filler message", LOG_COLOR_GEN_DEBUG2);
                    if (device_state->message_log_state == 1) { device_state->redraw = true; printf("filler message"); }
                }

                if (rightNew || leftNew) { 
                    device_state->message_log_state++;
                    device_state->message_log_state %= 2;
                    device_state->redraw = true;
                }

                if (device_state->msg_log_len > 10)
                if ((downNew || upNew) || (device_state->selector_delay > 30 && device_state->selector_delay % 3 == 0)) {
                    device_state->redraw = true;
                    if (downCur) 
                        if (device_state->msg_log_start == device_state->msg_log_len - 10) {
                            device_state->msg_log_start = 0;
                        }
                        else device_state->msg_log_start++;
                    else if (upCur) { 
                        if (device_state->msg_log_start == 0) {
                            device_state->msg_log_start = device_state->msg_log_len - 10;
                        }
                        else device_state->msg_log_start--;
                    }
                } 
            } break;
            case SCRNSTATE_MEM_DISPLAY: {
                if (!device_state->mem_viewscreen) {
                    if (downNew) {
                        device_state->current_screen = SCRNSTATE_CNFG_MAIN;
                        device_state->redraw = true;
                    } else if (upNew) {
                        device_state->mem_viewscreen = true;
                        device_state->redraw = true;
                    } else if (leftNew || rightNew) {
                        device_state->mem_sel_state++;
                        device_state->mem_sel_state %= 2;
                        device_state->redraw = true;
                    }

                    if (buttonNew) {
                        if (device_state->mem_sel_state == 0) {
                            if (device_state->mem_scr == 0) device_state->mem_scr = 23;
                            else device_state->mem_scr--;
                            device_state->redraw = true;
                        } else {
                            device_state->mem_scr++;
                            device_state->mem_scr %= 24;
                            device_state->redraw = true;
                        }
                    }
                } else {
                    bool autoMove = (device_state->selector_delay > 30 && device_state->selector_delay % 3 == 0);

                    if (rightNew || (autoMove && rightCur)) {
                        device_state->mem_mx++;
                        device_state->mem_mx %= 96;
                        device_state->redraw = true;
                    } else if (leftNew || (autoMove && leftCur)) {
                        if (device_state->mem_mx == 0) device_state->mem_mx = 95;
                        else device_state->mem_mx--;
                        device_state->redraw = true;
                    } else if (upNew || (autoMove && upCur)) {
                        if (device_state->mem_my == 0) device_state->mem_my = 57;
                        else device_state->mem_my--;
                        device_state->redraw = true;
                    } else if (downNew || (autoMove && downCur)) {
                        device_state->mem_my++;
                        device_state->mem_my %= 58;
                        device_state->redraw = true;
                    }

                    if (buttonNew) {
                        device_state->mem_viewscreen = false;
                        device_state->redraw = true;
                    }
                }

            } break;
        }

        if (syncSettings && device_state->wifi_state.con_state == CS_SERVER_CONNECTED) {
            send_setting_sync(device_state);
        }

    } else {
        // if sleeping and input, awaken
        if ((device_state->button_input > 0 || device_state->dinput > 0) && !remote) {

            // module likes to activate screensaver if left idle (on) for too long
            // best solution seems to be restarting it, since i cannot figure out how to otherwise disable the screensaver

            spi_deinit(spi_default);
            
            sleep_ms(1);

            init_oled_spi();

            device_state->sleeping = false;
            device_state->redraw = true;
        }
    }

    // do not bother with rendering if remote control
    if (remote) { 
        device_state->redraw = false;
        device_state->partial_redraw = false;
    }
    
}

void append_sample_log(struct State* device_state, uint16_t temp, uint16_t hum, uint32_t timestamp) {
    device_state->th_log_NEW[device_state->th_log_next].temp = temp;
    device_state->th_log_NEW[device_state->th_log_next].hum = hum;
    device_state->th_log_NEW[device_state->th_log_next].timestamp = timestamp;
    device_state->th_log_next++;
    if (device_state->th_log_next >= MAX_POLLS) device_state->th_log_next = 0;
    if (device_state->th_log_size < MAX_POLLS) device_state->th_log_size++;

    // add to wifi_queue
    device_state->th_log_wifi_queue[device_state->th_log_wifi_queue_size].temp = temp;
    device_state->th_log_wifi_queue[device_state->th_log_wifi_queue_size].hum = hum;
    device_state->th_log_wifi_queue[device_state->th_log_wifi_queue_size].timestamp = timestamp;
    if (device_state->th_log_wifi_queue_size < MAX_POLLS) device_state->th_log_wifi_queue_size++;

    if (/*!device_state->wifi_log_waiting_for_ack*/true) {
        device_state->th_log_wifi_queue_send_size = device_state->th_log_wifi_queue_size;
    }
}

void update_polls(struct State* device_state) {
    uint64_t curTime = get_absolute_time(); 
    // poll temperature
    if (device_state->temp_poll < curTime) {
        device_state->temp_poll = curTime + (device_state->poll_rate * 1000000);
        
        struct TH data = poll_TH();
        device_state->cur_TH = data;

        device_state->redraw = (device_state->current_screen == SCRNSTATE_HOME) || (device_state->current_screen == SCRNSTATE_GRAPH && device_state->logging_enabled) || device_state->redraw;

        // log poll
        if (device_state->logging_enabled) {
            append_sample_log(device_state, data.rawT, data.rawH, curTime / 1000000);
            
            if (device_state->wifi_state.con_state == CS_SERVER_CONNECTED/* && !device_state->wifi_log_waiting_for_ack*/) {
                add_log_msg(device_state, "sendlogdata()", LOG_COLOR_GEN_DEBUG3);
                send_log_data(device_state);
            }
        }
    }

    // check sleep delay
    if (device_state->sleep_poll < curTime && !device_state->sleeping) {
        device_state->sleeping = true;
        display_off();
    }

    if (device_state->sync_time && (curTime % 1000000) > 980000) {
        device_state->current_time.hour = device_state->sync_hour;
        device_state->current_time.min = device_state->sync_min;
        device_state->current_time.sec = device_state->sync_sec;
        rtc_set_datetime(&device_state->current_time);
        char msg[25];
        sprintf(msg, "TIME SYNC: %.2i;%.2i;%.2i", device_state->current_time.hour, device_state->current_time.min, device_state->current_time.sec);
        device_state->redraw = true;
        add_log_msg(device_state, msg, LOG_COLOR_PLAIN);
        device_state->sync_time = false;
    }

    if (device_state->current_screen == SCRNSTATE_MEM_DISPLAY) {
        device_state->mem_curstate += 2;
        device_state->mem_curstate %= 192;
        device_state->partial_redraw = true;
    }

    // update wifi stuff

    if (device_state->wifi_state.con_state != CS_OFF) {

        // do library things for async
        cyw43_arch_poll();

        // get tcp link status
        if (device_state->wifi_state.link_status_updly++ == 255) {
            int newStatus = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
            int newStatusWifi = cyw43_wifi_link_status(&cyw43_state, CYW43_ITF_STA);
            bool newTcp = newStatus != device_state->wifi_state.tcp_link_status;
            bool newWifi = newStatusWifi != device_state->wifi_state.wifi_link_status;

            if (newTcp || newWifi) {
                device_state->wifi_state.tcp_link_status = newStatus;
                device_state->wifi_state.wifi_link_status = newStatusWifi;

                // attempt to connect to server, now that it is connected to WAP
                if (newStatus == CYW43_LINK_UP) { 
                    device_state->wifi_state.con_state = CS_WAP_CONNECTED;
                    device_state->partial_redraw = true;
                    init_client(device_state);
                }

                // get signal strength immediately
                if (device_state->wifi_state.con_state == CS_WAP_CONNECTED || device_state->wifi_state.con_state == CS_SERVER_CONNECTED) { 
                    device_state->wifi_state.signal_strength_updly = 255;
                }

                if (newTcp) {
                    char str[18];
                    sprintf(str, "tcp: %s", cyw43_tcpip_link_status_name(newStatus));
                    add_log_msg(device_state, str, LOG_COLOR_LWIP_STATE);
                }
                
                if (newWifi) {
                    char str[19];
                    sprintf(str, "wifi: %s", cyw43_tcpip_link_status_name(newStatusWifi));
                    add_log_msg(device_state, str, LOG_COLOR_LWIP_STATE);
                }
            }
        }

        // animate wifi bars
        if (device_state->wifi_state.con_state == CS_CONNECTING) {
            if (device_state->wifi_state.bars_scanning_anim++ >= 180) device_state->wifi_state.bars_scanning_anim = 0;
            if (device_state->wifi_state.bars_scanning_anim % 30 == 0 && (device_state->current_screen == SCRNSTATE_HOME || device_state->current_screen == SCRNSTATE_CNFG_REMOTE_LOG)) device_state->partial_redraw = true;
        }
        
        // get signal strength
        if (device_state->wifi_state.con_state == CS_SERVER_CONNECTED || device_state->wifi_state.con_state == CS_WAP_CONNECTED)
        if (device_state->wifi_state.signal_strength_updly++ == 255) {
            int32_t rssi;
            cyw43_ioctl(&cyw43_state, 254, sizeof rssi, (uint8_t*)&rssi, CYW43_ITF_STA);
            uint8_t newStrength = (uint8_t)rssi;
            if (device_state->wifi_state.signal_strength != newStrength) {
                device_state->wifi_state.signal_strength = newStrength;

                /*
                char str[23];
                sprintf(str, "signal strength:%u", newStrength);
                add_log_msg(device_state, str, RGB(255, 255, 255));
                */
            }
        }

        // reconnect to server
        if (device_state->wifi_state.con_state == CS_WAP_CONNECTED && device_state->wifi_state.retry_server_connect) {
            device_state->wifi_state.retry_time_delay++;
            if (device_state->wifi_state.retry_time_delay >= CON_RETRY_DLY) {
                init_client(device_state);
                device_state->wifi_state.retry_server_connect = false;
                device_state->wifi_state.retry_time_delay = 0;
            }
            
        }

    }

    //device_state->wifi_state.prev_con_state = device_state->wifi_state.con_state;
    //device_state->wifi_state.prev_want_con = device_state->wifi_state.want_con;
}

// draw/render screen
void update_screen(struct State* device_state) {

    // do not write to display ram when screen is off
    if (device_state->sleeping) return;

    // draw new screen
    if (device_state->redraw || device_state->partial_redraw) {
        if (device_state->partial_redraw && device_state->redraw) device_state->partial_redraw = false;

        if (device_state->redraw) {
            clear_screen();
            sleep_ms(1);
        }

        switch (device_state->current_screen) {
            case SCRNSTATE_HOME: {
                if (!device_state->partial_redraw) {
                    uint16_t red = RGB(255, 0, 0);
                    uint16_t green = RGB(0, 255, 0);
                    uint16_t yellow = RGB(255, 255, 0);

                    char tempStr[8];
                    char humStr[6];
                    char timeStr[9];
                    char logStr[5];
                    char logCountStr[7];
                    const char* pa;
                    bool maxPolls = device_state->th_log_size >= MAX_POLLS;

                    rtc_get_datetime(&device_state->current_time);

                    if (device_state->current_time.hour >= 12) pa = "PM";
                    else pa = "AM";

                    int hour = getHour(&device_state->current_time);

                    sprintf(tempStr, "%.2f#F", rawT_to_float(device_state->cur_TH.rawT));
                    sprintf(humStr, "%.2f", rawH_to_float(device_state->cur_TH.rawH));
                    sprintf(timeStr, "%.2i;%.2i %s", hour, device_state->current_time.min, pa);
                    sprintf(logStr, "%s", maxPolls ? "FULL" : (device_state->logging_enabled ? "ON" : "OFF"));
                    sprintf(logCountStr, "(%i)", device_state->th_log_size);

                    draw_font5x3(timeStr, 63, 0, 0xFFFF, 0x0000);
                    draw_font10x13(tempStr, 13, 15, RGB(255, 255, 255));
                    draw_font10x13(humStr, 6, 38, RGB(255, 255, 255));
                    draw_font10x13("%RH", 60, 38, RGB(255, 255, 255));
                    draw_font5x3("LOG:", 1, 0, 0xFFFF, 0);
                    draw_font5x3(logStr, 17, 0, maxPolls ? red : (device_state->logging_enabled) ? green : RGB(255, 128, 0), 0);
                    draw_font5x3(logCountStr, maxPolls ? 33 : (device_state->logging_enabled) ? 25 : 29, 0, maxPolls ? red : 0xFFFF, 0);
                    draw_wifi_bars(device_state, 89, 59);
                } else {
                    draw_wifi_bars(device_state, 89, 59);
                }   
            } break;
            case SCRNSTATE_GRAPH: {
                // removing this check causes a div by zero. also makes no sense. only graph a flat line?
                if (device_state->th_log_size >= 2) {
                    // true = temperature
                    bool temperatureDisplayed = device_state->temp_graph;
                    //struct Poll_Node* curNode;
                    struct TH_Poll* currentSample;
                    double curSampleIdx;
                    //double prevSampleIdx;

                    // full redraw = normal
                    // partial redraw = do not determine min/max
                    if (!device_state->partial_redraw) {

                        // find highest T or H for selected range
                        uint16_t curMin = 65535;
                        uint16_t curMax = 0;
                        for (int i = device_state->zoom_start_idx; i <= device_state->zoom_end_idx; i++) {
                            uint16_t curValue = (temperatureDisplayed) ? device_state->th_log_NEW[i].temp : device_state->th_log_NEW[i].hum;

                            if (curValue < curMin) curMin = curValue;
                            if (curValue > curMax) curMax = curValue;
                        }

                        device_state->memo_graph_max = curMax;
                        device_state->memo_graph_min = curMin;

                    } else {
                        clear_screen();
                        sleep_ms(1);
                    }

                    uint8_t maxT = ceilf(temperatureDisplayed ? rawT_to_float(device_state->memo_graph_max) : rawH_to_float(device_state->memo_graph_max));
                    uint8_t minT = floorf(temperatureDisplayed ? rawT_to_float(device_state->memo_graph_min) : rawH_to_float(device_state->memo_graph_min));
                    uint8_t rangeT = maxT - minT;
                    draw_x_axis(0, maxT);
                    draw_x_axis(48, minT);

                    // debug
                    //double curIndex = 0.0;

                    uint8_t prevX = 0;
                    uint8_t prevY = 0;
                    struct TH_Poll* firstSample = NULL;
                    struct TH_Poll* lastSample = NULL;
                    int selectedSampleIdx = 0;
                    // without zooming, is always the entire sample bank
                    // totalSamplesToIterateThrough is just total_samples_drawn
                    
                    // graph_points_to_draw
                    int samplesToDraw = device_state->graph_points_to_draw;

                    // draw selection rectangle
                    if (device_state->graph_zoom_selecting) {
                        uint8_t rectStart = MIN(device_state->graph_selector_x, device_state->zoom_selection_x);
                        uint8_t rectEnd = MAX(device_state->graph_selector_x, device_state->zoom_selection_x);
                        draw_rectangle((uint8_t)floor(((double)(GRAPH_POINTS) / (samplesToDraw - 1)) * rectStart), 0, (uint8_t)floor(((double)GRAPH_POINTS / (samplesToDraw - 1)) * rectEnd), 49, RGBCMD(64, 32, 0), RGBCMD(64, 32, 0));
                        sleep_us(1000);
                    }

                    // draw axises
                    for (int i = 6; i < 43; i++) {
                        // ensures each axis will align with the line
                        float candNum = maxT - ((rangeT * i) / 48.0f);
                        if (fmodf(candNum, 0.1) > 0.099f) {
                            draw_x_axis(i, candNum);
                            i += 5;
                        }
                    }

                    // draw yellow/orange selector-thing underneath graphing lines
                    if (device_state->graph_select_mode) {
                        uint16_t selectorColor = (device_state->graph_zoom_selecting) ? RGBCMD(255, 128, 0) : RGBCMD(255, 255, 0);
                        uint8_t selector_x = floorf(((float)(GRAPH_POINTS - 1) / (samplesToDraw - 1)) * device_state->graph_selector_x);

                        draw_line(selector_x, 0, selector_x, 49, selectorColor);
                        draw_line((selector_x == 0) ? 0 : (selector_x - 1), 49, (selector_x == 0) ? 2 : selector_x + 1, 49, selectorColor);
                        //draw_font5x3("#", (selector_x == 0) ? 0 : (selector_x - 1), 50, RGB(255, 255, 0), 0);
                    }

                    curSampleIdx = device_state->zoom_start_idx;
                    firstSample = &(device_state->th_log_NEW[((int)round(curSampleIdx))]);
                    currentSample = NULL;
                    for (int i = 0; i < samplesToDraw; i++) {
                        
                        // curSampleIdx is not an integer raw index, have to round first
                        if (i != 0) {
                            curSampleIdx = device_state->zoom_start_idx + ((i / (double)(samplesToDraw - 1)) * (device_state->total_samples_drawn - 1));
                        }

                        currentSample = &(device_state->th_log_NEW[((int)round(curSampleIdx))]);
                        
                        uint8_t tempY;
                        
                        // read temp or hum data
                        if (temperatureDisplayed) {
                            tempY = (uint8_t)ceilf((rangeT - (rawT_to_float(currentSample->temp) - minT)) * (48.0f / rangeT));
                        } else {
                            tempY = (uint8_t)ceilf((rangeT - (rawH_to_float(currentSample->hum) - minT)) * (48.0f / rangeT));
                        }

                        uint8_t tempX = (uint8_t)floor(((double)(GRAPH_POINTS - 1) / (samplesToDraw - 1)) * i);
                        if (i != 0) draw_line(prevX, prevY, tempX, tempY, temperatureDisplayed ? RGBCMD(255, 0, 0) : RGBCMD(0, 255, 255));
                        prevX = tempX;
                        prevY = tempY;
                        if (i == device_state->graph_selector_x) {
                            device_state->graph_selector_sample = currentSample;
                            selectedSampleIdx = (int)round(curSampleIdx);
                            //curIndex = curSampleIdx;
                        }
                        if (i == samplesToDraw - 1) {
                            lastSample = currentSample;
                        }
                    }

                    //char dayStr[7];

                    uint32_t curTime = (uint32_t)(get_absolute_time() / 1000000);

                    if (!device_state->graph_select_mode) { 

                        draw_line(0, 49, 0, 56, RGBCMD(166, 255, 0));
                        draw_line(79, 49, 79, 56, RGBCMD(255, 166, 0));

                        if (temperatureDisplayed) draw_font5x3("TEMPERATURE", 21, 50, RGB(255, 150, 150), 0);
                        else draw_font5x3("HUMIDITY", 27, 50, RGB(150, 255, 255), 0);

                        uint8_t tempDay = 0;
                        uint8_t tempHour = 0;
                        uint8_t tempMin = 0;
                        // debug
                        uint8_t tempSec = 0;
                        getNormalTime(device_state, curTime, firstSample->timestamp, &tempDay, &tempHour, &tempMin, &tempSec);

                        char timeStr[13];
                        uint8_t hr = tempHour;
                        //uint8_t hr = device_state->th_log.polls[0].hour;
                        sprintf(timeStr, "%.2u;%.2u %s(%.2i)", hr % 12 == 0 ? 12 : hr % 12, tempMin, (hr >= 12) ? "PM" : "AM", tempDay % 100);
                        draw_font5x3(timeStr, 0, 59, RGB(166, 255, 0), 0);
                        
                        getNormalTime(device_state, curTime, lastSample->timestamp, &tempDay, &tempHour, &tempMin, NULL);

                        hr = tempHour;
                        sprintf(timeStr, "%.2u;%.2u %s(%.2i)", hr % 12 == 0 ? 12 : hr % 12, tempMin, (hr >= 12) ? "PM" : "AM", tempDay % 100);
                        draw_font5x3(timeStr, 49, 59, RGB(255, 166, 0), 0);
                    } else {
                        
                        char timeStr[27]; // if not debug, 9. lese, 27
                        char tempStr[9];
                        struct TH_Poll* sample = device_state->graph_selector_sample;
                        uint8_t day = 0;
                        uint8_t hr = 0;
                        uint8_t min = 0;
                        getNormalTime(device_state, curTime, sample->timestamp, &day, &hr, &min, NULL);

                        uint16_t selectionColor = RGB(255, 255, 0);
                        
                        sprintf(timeStr, "%.2u;%.2u %s(%.2i)", hr % 12 == 0 ? 12 : hr % 12, min, (hr >= 12) ? "PM" : "AM", day % 100);
                        //sprintf(dayStr, "DAY %u", sample->day);
                        if (temperatureDisplayed) sprintf(tempStr, "%.2f#F", rawT_to_float(sample->temp));
                        else sprintf(tempStr, "%.2f%%", rawH_to_float(sample->hum));
                        draw_font5x3(timeStr, 0, 59, selectionColor, 0);
                        draw_font5x3(tempStr, 48, 59, temperatureDisplayed ? RGB(255, 100, 100) : RGB(150, 255, 255), 0);
                        //draw_font5x3(dayStr, 6, 52, selectionColor, 0);

                        char idxStr[20];
                        // SAMPLE XXXX TO XXXX
                        // 20 (19 actual char)
                        sprintf(idxStr, "T%.4u (%.4u - %.4u)", device_state->total_samples_drawn, device_state->zoom_start_idx + 1, device_state->zoom_end_idx + 1);
                        draw_font5x3(idxStr, 0, 52, RGB(255, 128, 255), 0);
                        

                        char selX[4];
                        sprintf(selX, "%.4u", selectedSampleIdx + 1);
                        draw_font5x3(selX, 80, 54, RGB(255, 255, 64), 0);
                        sprintf(selX, "%.4u", device_state->th_log_size);
                        draw_font5x3(selX, 80, 59, RGB(128, 255, 128), 0);
                    }
                    
                } else {
                    draw_font5x3("NO LOGGING DATA TO SHOW", 2, 16, RGB(255, 255, 255), 0);
                    if (!device_state->logging_enabled) {
                        draw_font5x3("LOGGING IS DISABLED", 10, 32, RGB(255, 128, 0), 0);
                        draw_font5x3("ENABLE LOGGING IN", 14, 48, RGB(255, 255, 255), 0);
                        draw_font5x3("THE CONFIGURATION MENU", 4, 56, RGB(255, 255, 255), 0);
                    } else {
                        draw_font5x3("GRAPH REQUIRES AT LEAST", 2, 32, RGB(255, 255, 255), 0);
                        draw_font5x3("TWO DATA POINTS", 18, 38, RGB(255, 255, 255), 0);
                    }
                }
            } break;
            case SCRNSTATE_CNFG_MAIN: {
                uint8_t sel = device_state->setting_selector;
                //char pollRateStr[19];
                //bool log = device_state->logging_enabled;
                //sprintf(pollRateStr, "POLL RATE-%u SEC ", device_state->poll_rate);

                if (!device_state->partial_redraw) {
                    uint16_t cyan = RGB(0, 255, 255);
                    uint16_t white = 0xFFFF;

                    draw_config();
                    draw_font5x3("TIME", 3, 20, (sel == 0) ? cyan : white, 0x0000);
                    draw_font5x3("DISPLAY", 3, 28, (sel == 1) ? cyan : white, 0x0000);
                    draw_font5x3("POLLING", 3, 36, (sel == 2) ? cyan : white, 0x0000);
                    draw_font5x3("REMOTE LOG", 3, 44, (sel == 3) ? cyan : white, 0x0000);
                    draw_font5x3("MEMORY INSPECTOR", 3, 52, (sel == 4) ? cyan : white, 0x0000);
                    
                    draw_font5x3(VERSION, 76, 59, 0xFFFF, 0x0000);
                } else {
                    // TODO: are these even drawn here?
                    // for changing settings
                    switch (sel) {
                        case 2: {
                            //draw_font5x3(pollRateStr, 3, 36, RGB(0, 255, 255), 0x0000);
                        } break;
                        case 3: {
                            //draw_font5x3(log ? "LOGGING-ENABLED " : "LOGGING-DISABLED", 3, 44, RGB(0, 255, 255), 0x0000);
                        } break;
                    }
                }
                
            } break;
            case SCRNSTATE_CNFG_DISPLAY: {
                uint8_t sel = device_state->display_setting_selector;
                bool lowBrightness = device_state->display_brightness != 100;
                uint16_t cyan = RGB(0, 255, 255);
                uint16_t white = 0xFFFF;

                char brightnessStr[16];
                sprintf(brightnessStr, "BRIGHTNESS-%u%%", device_state->display_brightness);
                char sleepRateStr[14];
                sprintf(sleepRateStr, "SLEEP-%u SEC ", device_state->sleep_delay);

                if (!device_state->partial_redraw) {
                    draw_config();
                    draw_font5x3("DISPLAY", 34, 16, white, 0x0000);

                    draw_font5x3(brightnessStr, 3, 28, (sel == 0) ? cyan : white, 0x0000);
                    if (sel == 0 && lowBrightness) {
                        draw_rectangle(12, 35, 84, 55, RGBCMD(255, 255, 0), RGBCMD(32, 32, 32));
                        sleep_us(64);
                        draw_font5x3("SOME GRAPHICS MAY", 15, 37, RGB(255, 255, 0), RGB(32, 32, 32));
                        draw_font5x3("BE INVISIBLE AT", 20, 43, RGB(255, 255, 0), RGB(32, 32, 32));
                        draw_font5x3("LOW BRIGHTNESS", 21, 49, RGB(255, 255, 0), RGB(32, 32, 32));
                    } else {
                        draw_font5x3(sleepRateStr, 3, 36, (sel == 1) ? cyan : white, 0x0000);
                        draw_font5x3("BACK", 3, 58, (sel == 2) ? cyan : white, 0x0000);
                    }
                    
                    draw_font5x3(VERSION, 76, 59, 0xFFFF, 0x0000);
                } else {
                    // for changing settings
                    switch (sel) {
                        case 0: {
                            draw_font5x3(brightnessStr, 3, 28, RGB(0, 255, 255), 0x0000);
                        } break;
                        case 1: {
                            draw_font5x3(sleepRateStr, 3, 36, RGB(0, 255, 255), 0x0000);
                        } break;
                    }
                }
            } break;
            case SCRNSTATE_CNFG_TIME: {
                uint16_t cyan = RGB(0, 255, 255);
                uint16_t white = 0xFFFF;

                const char* pa;
                char hourStr[3];
                char minStr[3];
                
                if (device_state->time_temp_apm) pa = " AM";
                else pa = " PM";

                int hour = device_state->time_temp_hour;
                int minutes = device_state->time_temp_min;

                sprintf(hourStr, "%.2u", hour);
                sprintf(minStr, "%.2u", minutes);
                
                int sel = device_state->time_setting_selector;

                if (!device_state->partial_redraw) {
                    draw_config();
                    draw_font5x3("TIME", 40, 16, white, 0x0000);

                    // top arrows
                    draw_arrow(13, 23, false, sel == 0);
                    draw_arrow(43, 23, false, sel == 1);
                    draw_arrow(73, 23, false, sel == 2);

                    // text
                    draw_font10x13(hourStr, 8, 32, (sel == 0) ? cyan : white);
                    draw_font10x13(":", 28, 32, white);
                    draw_font10x13(minStr, 38, 32, (sel == 1) ? cyan : white);
                    draw_font10x13(pa, 58, 32, (sel == 2) ? cyan : white);
                    draw_font5x3("CONFIRM", 35, 59, (sel == 3) ? cyan : white, 0x0000);

                    // bottom arrows
                    draw_arrow(13, 48, true, sel == 0);
                    draw_arrow(43, 48, true, sel == 1);
                    draw_arrow(73, 48, true, sel == 2);

                } else {
                    // redraw new states
                    switch (sel) {
                        case 0: {
                            draw_font10x13(hourStr, 8, 32, cyan);
                        } break;
                        case 1: {
                            draw_font10x13(minStr, 38, 32, cyan);
                        } break;
                        case 2: {
                             draw_font10x13(pa, 58, 32, cyan);
                        } break;
                    }
                }

            } break;
            case SCRNSTATE_CNFG_POLL: {
                uint8_t sel = device_state->poll_setting_selector;
                uint16_t cyan = RGB(0, 255, 255);
                uint16_t white = 0xFFFF;
                uint16_t red = RGB(255, 0, 0);

                char pollRateStr[19];
                bool log = device_state->logging_enabled;
                bool maxlog = device_state->th_log_size >= MAX_POLLS;
                sprintf(pollRateStr, "POLL RATE-%u SEC ", device_state->poll_rate);

                if (!device_state->partial_redraw) {
                    draw_config();
                    draw_font5x3("POLLING", 34, 16, white, 0x0000);
                    
                    draw_font5x3(log ? "LOGGING-ENABLED " : "LOGGING-DISABLED", 3, 28, (sel == 0) ? cyan : white, 0x0000);
                    
                    draw_font5x3(pollRateStr, 3, 36, (sel == 1) ? cyan : white, 0x0000);
                    draw_font5x3("RESET LOG", 3, 44, (sel == 2) ? cyan : white, 0x0000);
                    draw_font5x3("BACK", 3, 58, (sel == 3) ? cyan : white, 0x0000);

                    draw_font5x3(VERSION, 76, 59, 0xFFFF, 0x0000);
                } else {
                    // for changing settings
                    switch (sel) {
                        case 0: {
                            draw_font5x3(log ? "LOGGING-ENABLED " : "LOGGING-DISABLED", 3, 28, cyan, 0x0000);
                        } break;
                        case 1: {
                            draw_font5x3(pollRateStr, 3, 36, cyan, 0x0000);
                        } break;
                    }
                }
            } break;
            case SCRNSTATE_CONFIRM: {
                uint8_t sel = device_state->conf_selector;
                uint16_t cyan = RGB(0, 255, 255);
                uint16_t white = 0xFFFF;

                if (!device_state->partial_redraw) {
                    draw_font5x3("ARE YOU SURE?", 21, 20, white, 0);
                    draw_font5x3("NO", 29, 35, (sel == 0) ? cyan : white, 0);
                    draw_font5x3("YES", 50, 35, (sel == 1) ? cyan : white, 0);
                } else {
                    draw_font5x3("NO", 29, 35, (sel == 0) ? cyan : white, 0);
                    draw_font5x3("YES", 50, 35, (sel == 1) ? cyan : white, 0);
                }

            } break;
            case SCRNSTATE_CNFG_REMOTE_LOG: {
                uint8_t sel = device_state->remote_log_setting_selector;
                uint16_t cyan = RGB(0, 255, 255);
                uint16_t white = 0xFFFF;
                uint16_t red = RGB(255, 0, 0);
                uint16_t testpollColor = (sel == 1) ? (device_state->wifi_state.con_state != CS_SERVER_CONNECTED) ? RGB(255, 0, 255) : cyan : (device_state->wifi_state.con_state != CS_SERVER_CONNECTED) ? red : white;

                if (!device_state->partial_redraw) {
                    draw_config();
                    draw_font5x3("REMOTE LOG", 28, 16, white, 0x0000);

                    char str[10];
                    sprintf(str, "WIFI: %s", (device_state->wifi_state.want_con) ? "ON" : "OFF");
                    draw_font5x3(str, 3, 28, (sel == 0) ? cyan : white, 0x0000);
                    draw_font5x3("TEST POLL", 3, 36, testpollColor, 0x0000);
                    draw_font5x3("BACK", 3, 58, (sel == 2) ? cyan : white, 0x0000);

                    draw_wifi_bars(device_state, 88, 54);
                    draw_font5x3(VERSION, 76, 59, 0xFFFF, 0x0000);
                } else {
                    switch (sel) {
                        // on/off
                        case 0: {
                            char str[10];
                            sprintf(str, "WIFI: %s", (device_state->wifi_state.want_con) ? "ON " : "OFF");
                            draw_font5x3(str, 3, 28, cyan, 0x0000);
                            break;
                        }
                        // debug poll
                        case 1: {
                            draw_font5x3("TEST POLL", 3, 36, testpollColor, 0x0000);
                            break;
                        }
                    }

                    draw_wifi_bars(device_state, 88, 54);
                }
            } break;
            case SCRNSTATE_DEBUG_LOG: {
                int offset = device_state->msg_log_start;
                uint16_t color = (device_state->msg_log_start == device_state->msg_log_len - 10) ? RGBCMD(64, 255, 255) : RGBCMD(255, 255, 255);
                sleep_ms(1);
                for (int i = 0; i < 10; i++) {
                    if (device_state->message_log_state == 0) draw_font5x3(device_state->message_log[i + offset].msg, 0, i * 6, device_state->message_log[i + offset].color, 0);
                    else {
                        const char* curStr = device_state->streamLog + ((offset + i) * 24);
                        size_t remLen = strlen(curStr);
                        char* strToDraw;
                        strncpy(strToDraw, curStr, 25);
                        while (remLen > 0 && i < 10) {
                            draw_font5x3(strToDraw, 0, i * 6, 0xffff, 0);
                            remLen -= 24;
                            i++;
                        }
                    }
                }
                // draw scroll line
                if (device_state->msg_log_len > 10) {

                    int median = (int)roundf(95 * (offset / (float)(device_state->msg_log_len - 10)));

                    draw_rectangle(0, 62, median, 63, color, color);
                    if (median != 95) draw_rectangle(median, 62, 95, 63, RGBCMD(64, 64, 64), RGBCMD(64, 64, 64));
                }
            } break;
            case SCRNSTATE_MEM_DISPLAY: {
                if (device_state->redraw) {
                    // 270336, 11136 on each
                    // ~23.7 screens
                    (270336 % 11136);// for last screen
                    
                    uint8_t screen = device_state->mem_scr;

                    char str[5];
                    sprintf(str, "%.2i", screen);
                    draw_font5x3(str, 0, 59, RGB(255, 255, 255), 0);

                    uint32_t memOffset = 0x20000000 + (screen * 11136);
                    
                    uint8_t lowerByte, upperByte;
                    lowerByte = *((uint8_t*)(memOffset + (device_state->mem_my * 96 * 2) + (device_state->mem_mx * 2)));
                    upperByte = *((uint8_t*)(memOffset + (device_state->mem_my * 96 * 2) + (device_state->mem_mx * 2) + 1));
                    sprintf(str, "%.2X%.2X", lowerByte, upperByte);
                    draw_font5x3(str, 12, 59, RGB(0, 255, 255), 0);

                    sprintf(str, "X:%.2i", device_state->mem_mx);
                    draw_font5x3(str, 32, 59, RGB(192, 0, 192), 0);
                    sprintf(str, "Y:%.2i", device_state->mem_my);
                    draw_font5x3(str, 52, 59, RGB(0, 192, 0), 0);

                    draw_mini_arrow(73, 59, false, (device_state->mem_sel_state == 0 && !device_state->mem_viewscreen));
                    draw_mini_arrow(83, 59, true, (device_state->mem_sel_state == 1 && !device_state->mem_viewscreen));

                    // is undone later, but if freezes, allows for easier identification
                    draw_point(95, 63, RGB(255, 0, 0));

                    if (screen == 23) write_display_data(0, 96, 0, 58, (uint8_t*)memOffset, 3072);
                    else write_display_data(0, 96, 0, 58, (uint8_t*)memOffset, 58 * 96 * 2);

                    draw_point(95, 63, RGB(0, 0, 0));
                } 

                // partial redraw & redraw. draws mouse cursor
                if (true) {
                    uint8_t mx = device_state->mem_mx;
                    uint8_t my = device_state->mem_my;
                    uint16_t curColor;
                    uint8_t curColorIt = device_state->mem_curstate;

                    // value shifting
                    curColor = RGB((8 * (curColorIt % 32) - 1), (8 * (curColorIt % 32) - 1), (8 * (curColorIt % 32) - 1));
                    
                    // draw left side 
                    if (mx != 0) draw_point(mx - 1, my, curColor);
                    // draw top side
                    if (my != 0) draw_point(mx, my - 1, curColor);
                    // draw right side
                    if (mx != 95) draw_point(mx + 1, my, curColor);
                    // draw bottom side
                    if (my != 57) draw_point(mx, my + 1, curColor);
                    // draw topleft
                    if (!(my == 0 || mx == 0)) draw_point(mx - 1, my - 1, curColor);
                    // draw topright
                    if (!(my == 0 || mx == 95)) draw_point(mx + 1, my - 1, curColor);
                    // draw bottomleft
                    if (!(my == 57 || mx == 0)) draw_point(mx - 1, my + 1, curColor);
                    // draw bottomright
                    if (!(my == 57 || mx == 95)) draw_point(mx + 1, my + 1, curColor);
                }
            }
        }
        device_state->redraw = false;
        device_state->partial_redraw = false;
    }
}

void add_wifi_err_log(struct State* device_state, const char* cbname, err_t err) {
    if (err != ERR_OK) {
        char str[25];
        sprintf(str, "%s ERROR: %s", cbname, lwip_err_name(err));
        add_log_msg(device_state, str, LOG_COLOR_LWIP_CONN_ERROR);
    }
}

void refresh_log_send_buffer(struct State* device_state) {

    // clear sent data
    for (int i = 0; i < device_state->th_log_wifi_queue_send_size; i++) {
        device_state->th_log_wifi_queue[i].hum = 0;
        device_state->th_log_wifi_queue[i].temp = 0;
        device_state->th_log_wifi_queue[i].timestamp = 0;
    }

    uint16_t newSize = 0;
    // copy(move) unsent data, chances are, this will rarely, if ever, be used.
    for (int i = device_state->th_log_wifi_queue_send_size, newIdx = 0; i < device_state->th_log_wifi_queue_size; i++, newIdx++, newSize++) {
        device_state->th_log_wifi_queue[newIdx].hum = device_state->th_log_wifi_queue[i].hum;
        device_state->th_log_wifi_queue[newIdx].temp = device_state->th_log_wifi_queue[i].temp;
        device_state->th_log_wifi_queue[newIdx].timestamp = device_state->th_log_wifi_queue[i].timestamp;
    }

    device_state->th_log_wifi_queue_size = newSize;
    device_state->th_log_wifi_queue_send_size = newSize;

    // do not need to clear remaining buffer, since it will be overwritten
}

void send_log_data(struct State* device_state) {
    send_data(device_state, ((uint8_t*)device_state->th_log_wifi_queue), (8 * device_state->th_log_wifi_queue_send_size), 'L', true);
    //device_state->wifi_log_waiting_for_ack = true;
    //device_state->wifi_log_waiting_for_ack_timeout = 0;
    
    add_log_msg(device_state, "log data sent", LOG_COLOR_GEN_DEBUG1);
    refresh_log_send_buffer(device_state);
}

void send_setting_sync(struct State* device_state) {
    uint8_t settings[4];
    settings[0] = device_state->display_brightness;
    settings[1] = device_state->sleep_delay;
    settings[2] = device_state->poll_rate;
    settings[3] = device_state->logging_enabled;
    send_data(device_state, settings, 4, 'E', true);
}

// appends data size to front of buffer. 2 bytes. dataLen is the size of the passed in data, excluding pIdent
void send_data(struct State* device_state, uint8_t* data, uint16_t dataLen, uint8_t pIdent, bool immediate) {

    // the 2 byte int size includes the first letter packet identifier

    uint8_t* fullDataPtr = malloc(dataLen + 3);
    fullDataPtr[0] = (dataLen + 1) << 8;
    fullDataPtr[1] = (uint8_t)(dataLen + 1);
    fullDataPtr[2] = pIdent;
    
    if (dataLen != 0) memcpy(&fullDataPtr[3], data, dataLen);

    tcp_write(device_state->wifi_state.tcp_pcb, fullDataPtr, dataLen + 3, TCP_WRITE_FLAG_COPY);
    if (immediate) tcp_output(device_state->wifi_state.tcp_pcb);
}

void print_packet(struct State* device_state, uint8_t* data, uint16_t totlen, uint16_t msgColor) {
    // print out up to 64 bytes of the request
    char str[25];
    // initialize string
    strcpy(str, "\0");
    char tempStr[4];
    for (int i = 0, line = 0; i < ((totlen > 64) ? 64 : totlen); i++, line++) {
        if (line >= 8) {
            add_log_msg(device_state, str, msgColor);
            // empty string buffer for new allocation
            strcpy(str, "\0");
            line = 0;
        }
        sprintf(tempStr, "%.2X ", *(data + i));
        strcat(str, tempStr);
    }
    add_log_msg(device_state, str, msgColor);
}

void handle_recv_data(struct State* device_state, uint8_t* data, uint16_t len, uint16_t totlen) {
    // do not disconnect if server alive
    device_state->wifi_state.time_out = 0;
    //add_log_msg(device_state, "connection spared", LOG_COLOR_GEN_DEBUG1);

    // packet data contains:
    // 16-bit int; length of data
    // first char is type
    int32_t dataLeft = len;
    uint16_t dataCur = 0;

    //uint8_t safety = 0;

    if (len == 0 && totlen > 0) {
        add_log_msg(device_state, "likely server disconnect", LOG_COLOR_CONN_INFO);
        send_data(device_state, "what", 5, '_', true);
    }

    // TODO: support data being spliced among multiple packets

    while (dataLeft > 0) {
        uint16_t pckSize = (data[dataCur] << 8) | data[dataCur + 1];

        // increment past size int
        dataCur += 2;

        // valid packet types
        /*
        C - control scheme packet. contains list of user controls to remotely execute
        D - return current screen data
        E - setting sync, sync settings from server
        L - NOT USED receive acknowledgement of uploaded log data 
        S - return current timestamp in seconds
        T - time sync, syncs with server's time
        
        */
        
        switch (data[dataCur]) {
            // change time via time sync with server
            case 'T': {
                device_state->sync_time = true;
                device_state->sync_hour = data[dataCur + 1];
                device_state->sync_min = data[dataCur + 2];
                device_state->sync_sec = data[dataCur + 3];
            } break;

            // set settings to server's
            case 'E': {
                device_state->display_brightness = data[dataCur + 1];
                device_state->sleep_delay = data[dataCur + 2];
                device_state->poll_rate = data[dataCur + 3];
                device_state->logging_enabled = data[dataCur + 4];

                // safety checks for sleep delay and poll rate
                if (device_state->sleep_delay >= 30) device_state->sleep_delay = 30;
                if (device_state->poll_rate <= 10) device_state->poll_rate = 10;

                device_state->redraw = true; // not partial_redraw; set_display_brightness appears to clear the display
                set_display_brightness(device_state->display_brightness * 2.55);
                sleep_ms(1);

                add_log_msg(device_state, "setting sync received", LOG_COLOR_PLAIN);
            } break;

            case 'L': {
                break;
                /*
                uint16_t receivedSize = (data[dataCur + 1] << 8) | data[dataCur + 2];
                char buf[25];
                sprintf(buf, "recv send size: %u", receivedSize);
                add_log_msg(device_state, buf, LOG_COLOR_GEN_DEBUG1);
                // if successfully sent
                if (device_state->th_log_wifi_queue_send_size == receivedSize / 8) {
                    refresh_log_send_buffer(device_state);
                    add_log_msg(device_state, "successful log sent", LOG_COLOR_GEN_DEBUG1);
                } else {
                    // nothing for now
                }
                */
            } break;

            // remote control!
            case 'C': {
                // make copy of input state buffers
                uint8_t dinput_c = device_state->dinput;
                uint8_t button_input_c = device_state->button_input;
                uint8_t aux_button_input_c = device_state->aux_button_input;

                // reset temporairily
                device_state->dinput = 0;
                device_state->button_input = 0;
                device_state->aux_button_input = 0;

                //add_log_msg(device_state, "remote Ctrl recv", LOG_COLOR_GEN_DEBUG1);

                // TODO: make end condition based on pckSize
                for (int i = 1; data[dataCur + i] != '\0'; i++) {
                    // reset temporairily
                    device_state->dinput = 0;
                    device_state->button_input = 0;
                    device_state->aux_button_input = 0;

                    switch (data[dataCur + i]) {
                        // joystick up
                        case 'U': {
                            device_state->dinput |= 4;
                        } break;
                        // joystick down
                        case 'D': {
                            device_state->dinput |= 1;
                        } break;
                        // joystick right
                        case 'R': {
                            device_state->dinput |= 2;
                        } break;
                        // joystick left
                        case 'L': {
                            device_state->dinput |= 8;
                        } break;
                        // joystick button
                        case 'B': {
                            device_state->button_input |= 1;
                            //add_log_msg(device_state, "button press received", LOG_COLOR_GEN_DEBUG1);
                        } break;
                        // auxiliary/debug button
                        case 'A': {
                            device_state->aux_button_input |= 1;
                        } break;
                        default: goto superbreak;
                    }
                    handle_input(device_state, true);
                    // iterate/flush input buffers
                    // it is done completely, because if the same input is done immediately twice, it does not recognize it as an input 
                    //device_state->dinput <<= 8;
                    //device_state->button_input <<= 8;
                    //device_state->aux_button_input <<= 8;
                }
                superbreak:

                // set back to original
                device_state->dinput = dinput_c;
                device_state->button_input = button_input_c;
                device_state->aux_button_input = aux_button_input_c;

                
            } break;

            // respond to screen data request (not working currently, can't figure out exact spi command)
            case 'D': {
                
                // debug
                char str[25];
                sprintf(str, "sndbuf: %.2u", tcp_sndbuf(device_state->wifi_state.tcp_pcb));
                add_log_msg(device_state, str, LOG_COLOR_PLAIN);

                uint8_t* screenData = malloc(12289);
                screenData[0] = 'D';
                read_display_data(screenData + 1);
                err_t er = tcp_write(device_state->wifi_state.tcp_pcb, screenData, 12289, TCP_WRITE_FLAG_COPY);
                free(screenData);

                if (er != ERR_OK) add_wifi_err_log(device_state, "write", er);

                add_log_msg(device_state, "display data sent", LOG_COLOR_PLAIN);
                break;
            }

            // response to timeStamp request
            case 'S': {
                uint8_t buf[5];
                uint32_t curTime = (uint32_t)(get_absolute_time() / 1000000);
                buf[0] = *(uint8_t*)(&curTime);
                buf[1] = *(((uint8_t*)(&curTime)) + 1);
                buf[2] = *(((uint8_t*)(&curTime)) + 2);
                buf[3] = *(((uint8_t*)(&curTime)) + 3);

                send_data(device_state, buf, 4, 'S', true);
            } break;

            default: {
                add_log_msg(device_state, "UNKNOWN RECV DATA:", LOG_COLOR_GEN_ERROR);
                print_packet(device_state, data, totlen, LOG_COLOR_GEN_ERROR);

                // send data logging unknown data. if no response (rst packet or similar) then it knows the connection has ended
                send_data(device_state, "unknown data received", 22, '_', true);
            } break;
        }
        
        dataLeft -= pckSize + 2;
        dataCur += pckSize;
    }
}

err_t close_connection(void* arg) {
    struct State* device_state = (struct State*)(arg);
    struct Wifi_State* ws = &device_state->wifi_state;
    if (ws->tcp_pcb != NULL) {
        tcp_arg(ws->tcp_pcb, NULL);
        tcp_poll(ws->tcp_pcb, NULL, 0);
        tcp_sent(ws->tcp_pcb, NULL);
        tcp_recv(ws->tcp_pcb, NULL);
        tcp_err(ws->tcp_pcb, NULL);
        tcp_close(ws->tcp_pcb);
        ws->tcp_pcb = NULL;
        add_log_msg(device_state, "client closed", LOG_COLOR_LWIP_INFO);
        device_state->wifi_state.con_state = CS_WAP_CONNECTED;
    } 

    return ERR_OK;
}

err_t server_connected_cb(void* arg, struct tcp_pcb* pcb, err_t err) {
    struct State* device_state = (struct State*)(arg);

    add_log_msg(device_state, "SERVER CONNECTED", LOG_COLOR_CONN_INFO);
    add_wifi_err_log(device_state, "cond", err);

    device_state->wifi_state.con_state = CS_SERVER_CONNECTED;

    // sync settings and time
    send_data(device_state, NULL, 0, 'Z', false);

    if (device_state->th_log_wifi_queue_send_size > 0) send_log_data(device_state);

    if (device_state->current_screen == SCRNSTATE_HOME || device_state->current_screen == SCRNSTATE_CNFG_REMOTE_LOG) device_state->partial_redraw = true;

    return ERR_OK;
}

void handle_connection_error_cb(void* arg, err_t err) {
    struct State* device_state = (struct State*)(arg);
    struct Wifi_State* ws = &(device_state->wifi_state);

    if (err != ERR_ABRT) {
    }

    // if failed to connect to server
    if ((err == ERR_ABRT || err == ERR_RST) && (ws->con_state == CS_WAP_CONNECTED || ws->con_state == CS_SERVER_CONNECTED)) {
        ws->retry_server_connect = true;
    }

    add_wifi_err_log(device_state, "cerr", err);

    close_connection(arg);
}

err_t receive_data_cb(void* arg, struct tcp_pcb* pcb, struct pbuf* pb, err_t err) {
    struct State* device_state = (struct State*)(arg);
    if (pb->len != 1) {
        char str[25];
        sprintf(str, "RECV: %u %u", pb->len, pb->tot_len);
        add_log_msg(device_state, str, LOG_COLOR_LWIP_INFO);
    }
    
    // debug
    /* 
    for (int i = 0; i < pb->len; i++) {
        sprintf(str, "%x", *((uint8_t*)(pb->payload) + i));
        add_log_msg(device_state, str, RGB(0, 255, 64));
    }
    */

    handle_recv_data(device_state, pb->payload, pb->len, pb->tot_len);

    add_wifi_err_log(device_state, "recv", err);

    return ERR_OK;
}

err_t data_sent_cb(void* arg, struct tcp_pcb* pcb, uint16_t len) {
    struct State* device_state = (struct State*)(arg);
    
    if (len != 1) {
        char str[25];
        sprintf(str, "SENT: %u", len);
        add_log_msg(device_state, str, LOG_COLOR_LWIP_INFO);
    }

    return ERR_OK;
}

// generally this isn't needed, as the server sends tcp keep-alives
err_t poll_connection_cb(void* arg, struct tcp_pcb* pcb) {
    struct State* device_state = (struct State*)(arg);
    
    /*
    add_log_msg(device_state, "CON POLLED", LOG_COLOR_GEN_DEBUG1);
    uint8_t query = 'A';
    err_t res = tcp_write(device_state->wifi_state.tcp_pcb, &query, 1, TCP_WRITE_FLAG_COPY);
    if (res != ERR_OK) {
        add_wifi_err_log(device_state, "CONW", res);
    } else {
        res = tcp_output(pcb);
        if (res != ERR_OK) {
            add_wifi_err_log(device_state, "CONO", res);
        }
    }
    */
   
    return ERR_OK;
}

void init_client(struct State* device_state) {
    // prevent many connections
    close_connection(device_state);

    struct Wifi_State* ws = &device_state->wifi_state;
    
    // make new pcb
    ws->tcp_pcb = tcp_new_ip_type(IP_GET_TYPE(&ws->addr));
    
    // set callbacks
    tcp_arg(ws->tcp_pcb, device_state); // set arguments for callbacks (device_state in this case)
    tcp_poll(ws->tcp_pcb, poll_connection_cb, CON_POLL_DLY * 2);
    tcp_sent(ws->tcp_pcb, data_sent_cb); // called when data is sent?
    tcp_recv(ws->tcp_pcb, receive_data_cb); // recieve data from server?
    tcp_err(ws->tcp_pcb, handle_connection_error_cb); // handle errors

    cyw43_arch_lwip_begin();
    tcp_connect(device_state->wifi_state.tcp_pcb, &device_state->wifi_state.addr, SERVER_PORT, server_connected_cb);
    cyw43_arch_lwip_end();
    add_log_msg(device_state, "connection attempting", LOG_COLOR_CONN_INFO);
}

void init_wifi(struct State* device_state) {

    struct Wifi_State* ws = &device_state->wifi_state;

    cyw43_arch_init();
    cyw43_arch_enable_sta_mode();
    
    // connect to router
    cyw43_arch_wifi_connect_bssid_async(WIFI_SSID, NULL, WIFI_PW, CYW43_AUTH_WPA2_AES_PSK);
    //cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PW, CYW43_AUTH_WPA2_AES_PSK, 30000);

    // does not need to be recalled
    ip4addr_aton(WIFI_SERVER_IP, &ws->addr);

    ws->con_state = CS_CONNECTING;
    add_log_msg(device_state, "wifi turned on", LOG_COLOR_CONN_INFO);

    // https://github.com/raspberrypi/pico-examples/blob/master/pico_w/wifi/tcp_client/picow_tcp_client.c
}

void turn_wifi_off(struct State* device_state) {
    close_connection(device_state);
    device_state->wifi_state.con_state = CS_OFF;
    add_log_msg(device_state, "wifi turned off", LOG_COLOR_CONN_INFO);
    cyw43_arch_disable_sta_mode();
    cyw43_arch_deinit();
}

datetime_t get_next_minute(const datetime_t* dt) {
    datetime_t nextMinute = {
        .year = 0,
        .month = 1,
        .day = dt->day,
        .dotw = dt->dotw,
        .hour = dt->hour,
        .min = dt->min,
        .sec = dt->sec
    };

    nextMinute.sec = 0;
    nextMinute.min++;

    if (nextMinute.min == 60) {
        nextMinute.min = 0;
        nextMinute.hour++;
        if (nextMinute.hour == 24) {
            nextMinute.hour = 0;
            nextMinute.day++;
        }
    }

    return nextMinute;
}

// used for these callbacks only, which take no parameters. otherwise points towards the device_state
struct State* global_device_state;
void inc_time() {
    global_device_state->redraw = (global_device_state->current_screen == SCRNSTATE_HOME) || (global_device_state->current_screen == SCRNSTATE_GRAPH) || global_device_state->redraw;

    datetime_t nextAlarm = get_next_minute(&global_device_state->current_time);
    rtc_set_alarm(&nextAlarm, inc_time);
    rtc_enable_alarm();
}

void reset_temp_log() {
    global_device_state->th_log_size = 0;
    global_device_state->th_log_next = 0;
    global_device_state->logging_enabled = false;
    global_device_state->graph_selector_x = 0;

    // only used for the log, so reset here
    global_device_state->current_time.day = 1;
}

void add_log_msg(struct State* device_state, const char* msg, uint16_t color) {
    
    if (device_state->current_screen == SCRNSTATE_DEBUG_LOG) device_state->redraw = true;

    for (int i = 0; i < LOG_LENGTH; i++) {
        if (strcmp(device_state->message_log[i].msg, "\0") == 0) {
            strcpy(device_state->message_log[i].msg, msg);
            device_state->message_log[i].color = color;
            if (device_state->msg_log_len < LOG_LENGTH) { 
                bool lock = false;
                if (device_state->msg_log_len > 10 && device_state->msg_log_start == device_state->msg_log_len - 10) lock = true; 
                device_state->msg_log_len++;
                if (lock) device_state->msg_log_start = device_state->msg_log_len - 10; // ++
            }
            return;
        }
    }

    // move all messages forward by one
    for (int i = 0; i < LOG_LENGTH - 1; i++) {
        strcpy(device_state->message_log[i].msg, device_state->message_log[i + 1].msg);
        device_state->message_log[i].color = device_state->message_log[i + 1].color;
    }

    strcpy(device_state->message_log[LOG_LENGTH - 1].msg, msg);
    device_state->message_log[LOG_LENGTH - 1].color = color;
}

int main() {
    stdio_init_all();

    // init SHT via i2c
    i2c_init(i2c0, 100 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    // for joystick
    adc_init();
    adc_gpio_init(X_PIN);
    adc_gpio_init(Y_PIN);
    adc_gpio_init(BUTTON_PIN);
    // adc_select_input() must be used before reading from different pins

    // for aux button
    gpio_init(AUX_BUTTON_PIN);
    gpio_set_dir(AUX_BUTTON_PIN, GPIO_IN);
    gpio_pull_down(AUX_BUTTON_PIN);

    init_oled_spi();

    struct Wifi_State wifi_state = {
        //.prev_want_con = false,
        .want_con = true,
        .signal_strength = 0,
        .signal_strength_updly = 0, 
        .bars_scanning_anim = 0,
        .con_state = CS_CONNECTING,
        //.prev_con_state = CS_OFF,
        .tcp_link_status = -8,
        .wifi_link_status = -8,
        .link_status_updly = 0,
        .tcp_pcb = NULL,
        .retry_server_connect = false
    };

    struct State device_state = {
        .current_screen = SCRNSTATE_HOME,
        .back_current_screen = SCRNSTATE_HOME,
        .current_time = {
            .year = 0,
            .month = 1,
            .day = 1,
            .dotw = 0,
            .hour = 0,
            .min = 0,
            .sec = 0
        },
        .dinput = 0,
        .button_input = 0,
        .display_brightness = 100,
        .display_brightness_index = 3,
        .poll_rate = 60,
        .poll_rate_index = 3,
        .sleep_delay = 20,
        .sleep_delay_index = 2,
        .temp_poll = 0,
        .sleep_poll = (20 * 1000000),
        .selector_delay = 0,
        .display_setting_selector = 0,
        .setting_selector = 0,
        .remote_log_setting_selector = 0,
        .graph_select_mode = false,
        .graph_selector_x = 0,
        .redraw = false,
        .partial_redraw = false,
        .logging_enabled = true,
        .temp_graph = true,
        .graph_zoom_selecting = false,
        .zoom_selection_x = 0,
        .zoom_start_idx = 0,
        .zoom_end_idx = 1,
        .total_samples_drawn = 2,
        .graph_points_to_draw = 2,
        .memo_graph_max = 0,
        .memo_graph_min = 65535,
        .msg_log_start = 0,
        .msg_log_len = 0,
        .wifi_state = wifi_state,
        .sync_time = false,
        .sync_hour = 0,
        .sync_min = 0,
        .sync_sec = 0,
        .th_log_next = 0,
        .th_log_size = 0,
        .mem_mx = 0,
        .mem_my = 0,
        .mem_curstate = 0,
        .mem_scr = 0,
        .mem_sel_state = 0,
        .mem_viewscreen = false,
        .th_log_wifi_queue_size = 0,
        .th_log_wifi_queue_send_size = 0
    };

    // init msg log
    for (int i = 0; i < LOG_LENGTH; i++) {
        strcpy(device_state.message_log[i].msg, "\0");
        device_state.message_log[i].color = 0x0000;
    }

    // init sample log
    device_state.th_log_NEW = malloc(sizeof(struct TH_Poll) * MAX_POLLS);
    device_state.th_log_wifi_queue = malloc(sizeof(struct TH_Poll) * MAX_POLLS);

    // init stream log

    global_device_state = &device_state;

    set_display_brightness(device_state.display_brightness * 2.55);

    rtc_init();
    rtc_set_datetime(&device_state.current_time);
    datetime_t nextMinute = get_next_minute(&device_state.current_time);
    rtc_set_alarm(&nextMinute, inc_time);

    absolute_time_t sleep_til = get_absolute_time(); 
    
    // init wifi
    init_wifi(&device_state);

    while (true)
    {
        sleep_til = get_absolute_time() + 20000; // 20ms

        read_input(&device_state);
        update_polls(&device_state);
        handle_input(&device_state, false);
        update_screen(&device_state);
        sleep_until(sleep_til);
    }

    return 0;
}