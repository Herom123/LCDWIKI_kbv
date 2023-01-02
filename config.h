//you can copy this to your header file and include before LCDWIKI_KBV - then - these should be overwritten
#ifndef CUSTOMIZE_CONFIG
#define CUSTOMIZE_CONFIG

//if using 8bit mode,set the below macro definition to 1
//if using 16bit mode,set the below macro definition to 0
#define CONFIG_USE_8BIT_BUS 0

//if using 8bit mode on Mega2560 and the data pin is from 22 to 29,please uncomment the below macro definition and set it to 1
//if using 8bit mode on Mega2560 and the data pin is from 30 to 37,please uncomment the below macro definition and set it to 0
//if using 8bit mode on UNO or Mega2560 and the data pin is from 2 to 9,please comment the below macro definition
//#define USE_8BIT_SHIELD_ON_MEGA 0


////*===================================================*\\\\
//                          ESP32                          \\
////*===================================================*\\\\
                      *16bit mode only

#if defined(ESP32) && (CONFIG_USE_8BIT_BUS==0)
    //ALL DATA-BUS PINS MUST BE UNDER 32 
    #define TFT_D0   15 
    #define TFT_D1   2 
    #define TFT_D2   0
    #define TFT_D3   4 
    #define TFT_D4   16
    #define TFT_D5   17
    #define TFT_D6   5
    #define TFT_D7   18
    #define TFT_D8   19  
    #define TFT_D9   21 
    #define TFT_D10  22  
    #define TFT_D11  23  
    #define TFT_D12  13
    #define TFT_D13  12
    #define TFT_D14  14
    #define TFT_D15  27
    // RD and WR pins must be under 32
    #define TFT_CS   33
    #define TFT_RS   26 
    #define TFT_WR   25
    #define TFT_RD   -1
    #define TFT_RST  32
#endif

#endif // _mcu_pin_magic_
